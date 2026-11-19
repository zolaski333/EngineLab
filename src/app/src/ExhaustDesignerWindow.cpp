#include <enginelab/app/ExhaustDesignerWindow.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/foundation/CatalystMonolithGeometry.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace enginelab {
namespace {

constexpr std::size_t maximumComponents = 256;

[[nodiscard]] juce::String utf8(std::string_view value) {
    return juce::String::fromUTF8(value.data(), static_cast<int>(value.size()));
}

/** Smallest duct/junction CFL length in a compiled exhaust; 0 if unknown.
 *
 * Validation checks ranges and topology and nothing else, so a network that is
 * perfectly legal can still be ruinously expensive: the explicit solver's time
 * step is set by the minimum duct dx or junction 2 V/sum(A), and every other
 * duct then substeps at that rate. A user who rebuilt a 2JZ exhaust with an
 * 80 x 10 mm silencer body reported that the engine had "gained inertia, lost
 * its liveliness and made almost no sound" -- three descriptions of one cause,
 * the network going from 11,520 Hz to 92,160 Hz of substepping and the physics
 * thread landing at 153 % of its frame budget. Nothing in the editor could
 * have told them.
 *
 * Returning the geometric length rather than a frequency is deliberate: the
 * substep rate is proportional to its inverse at the same gas state, without
 * assuming a fixed sound speed in the editor.
 */
[[nodiscard]] double minimumExhaustCflLengthM(const EngineConfig& config) {
    try {
        const auto graph = ExhaustGraph::makeForEngine(config);
        // At the engine's own production mesh (100 mm on a two-stroke).
        auto mesh = gasdynamics::realtimeExhaustFeedbackDiscretisation();
        mesh.targetCellLengthM = EngineSimulator::productionExhaustCellLengthM(config);
        const auto layout = gasdynamics::ExhaustNetworkLayout::compile(graph, mesh);
        if (!layout.valid()) return 0.0;
        return layout.minimumCflLengthM();
    } catch (...) {
        return 0.0;
    }
}

[[nodiscard]] const char* componentTypeName(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe: return "Pipe";
    case ExhaustComponentType::merge: return "Collector";
    case ExhaustComponentType::splitter: return "Splitter";
    case ExhaustComponentType::resonator: return "Resonator";
    case ExhaustComponentType::muffler: return "Muffler";
    case ExhaustComponentType::catalyst: return "Catalyst";
    case ExhaustComponentType::outlet: return "Outlet";
    case ExhaustComponentType::crossover: return "X-pipe";
    }
    return "Unknown";
}

[[nodiscard]] juce::Colour componentTypeColour(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe: return juce::Colour(0xff4f8a78);
    case ExhaustComponentType::merge: return juce::Colour(0xffd28a42);
    case ExhaustComponentType::splitter: return juce::Colour(0xff8d75c5);
    case ExhaustComponentType::resonator: return juce::Colour(0xff4b88b8);
    case ExhaustComponentType::muffler: return juce::Colour(0xffb16a71);
    case ExhaustComponentType::catalyst: return juce::Colour(0xffa19b55);
    case ExhaustComponentType::outlet: return juce::Colour(0xff6d7472);
    case ExhaustComponentType::crossover: return juce::Colour(0xff4f90c9);
    }
    return juce::Colour(0xff6d7472);
}

[[nodiscard]] bool parseFinite(const juce::String& text, double& value) {
    try {
        std::size_t consumed = 0;
        const auto source = text.trim().toStdString();
        value = std::stod(source, &consumed);
        return consumed == source.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

[[nodiscard]] bool parseId(const juce::String& text, std::uint32_t& value) {
    try {
        std::size_t consumed = 0;
        const auto source = text.trim().toStdString();
        const auto parsed = std::stoull(source, &consumed, 10);
        if (consumed != source.size() || parsed == 0
            || parsed > std::numeric_limits<std::uint32_t>::max()) return false;
        value = static_cast<std::uint32_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

class CallbackListModel final : public juce::ListBoxModel {
public:
    using Count = std::function<int()>;
    using Text = std::function<juce::String(int)>;
    using Select = std::function<void(int)>;

    CallbackListModel(Count count, Text text, Select select)
        : count_(std::move(count)), text_(std::move(text)), select_(std::move(select)) {}

    int getNumRows() override { return count_ ? count_() : 0; }

    void paintListBoxItem(int row, juce::Graphics& graphics, int width, int height,
                          bool selected) override {
        if (selected) graphics.fillAll(juce::Colour(0xff334c45));
        graphics.setColour(selected ? juce::Colour(0xffffc56d) : juce::Colour(0xffd4dcda));
        graphics.setFont(juce::FontOptions(13.0F, selected ? juce::Font::bold : 0));
        graphics.drawText(text_ ? text_(row) : juce::String {}, 8, 0, width - 12, height,
                          juce::Justification::centredLeft, true);
    }

    void selectedRowsChanged(int row) override {
        if (select_) select_(row);
    }

private:
    Count count_;
    Text text_;
    Select select_;
};

class ExhaustGraphCanvas final : public juce::Component, public juce::SettableTooltipClient {
public:
    using PathProvider = std::function<const ExhaustPathConfig*()>;
    using SelectionProvider = std::function<std::uint32_t()>;
    using SelectionCallback = std::function<void(std::uint32_t)>;
    /** A face's rotor (from 0), or -1 for a piston engine's cylinder. */
    using RotorOf = std::function<int(std::uint32_t)>;

    ExhaustGraphCanvas(PathProvider pathProvider, SelectionProvider selectionProvider,
                       SelectionCallback selectionCallback, RotorOf rotorOf)
        : pathProvider_(std::move(pathProvider)), selectionProvider_(std::move(selectionProvider)),
          selectionCallback_(std::move(selectionCallback)), rotorOf_(std::move(rotorOf)) {
        setWantsKeyboardFocus(true);
    }

    /** A node dragged onto another: connect them. */
    std::function<void(std::uint32_t, std::uint32_t)> onConnectComponents;
    /** A source (a cylinder, or a rotor's faces) dragged onto a node. */
    std::function<void(const std::vector<std::uint32_t>&, std::uint32_t)> onConnectCylinders;
    /** Right-click on a node, or on a source. */
    std::function<void(std::uint32_t)> onComponentMenu;
    std::function<void(const std::vector<std::uint32_t>&)> onSourceMenu;

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour(0xff101716));
        graphics.setColour(juce::Colour(0xff25312e));
        graphics.drawRect(getLocalBounds(), 1);
        nodeBounds_.clear();
        sourceBounds_.clear();

        const auto* path = pathProvider_ ? pathProvider_() : nullptr;
        if (path == nullptr || !path->network || path->network->components.empty()) {
            graphics.setColour(juce::Colour(0xff83908c));
            graphics.setFont(juce::FontOptions(15.0F));
            graphics.drawFittedText("No custom network. Generate the legacy network or add components.",
                                    getLocalBounds().reduced(30), juce::Justification::centred, 3);
            return;
        }

        const auto& network = *path->network;
        // The sources down the left: a cylinder each, or a rotor each. A
        // rotor's three faces open in turn onto one port, so it is drawn as
        // one source, and the passage only its faces feed (the merge the
        // simulator sums them in) is its port, drawn inside it: three
        // exhausts a rotor made no sense (owner report 2026-10-30).
        std::vector<juce::String> sourceLabels;
        std::unordered_map<std::uint32_t, std::size_t> sourceRow;
        for (const auto cylinderId : path->cylinderIds) {
            const auto rotor = rotorOf_ ? rotorOf_(cylinderId) : -1;
            const auto label = rotor >= 0 ? "ROTOR " + juce::String(rotor + 1) : "CYL " + juce::String(cylinderId);
            const auto found = std::find(sourceLabels.begin(), sourceLabels.end(), label);
            sourceRow[cylinderId] = static_cast<std::size_t>(std::distance(sourceLabels.begin(), found));
            if (found == sourceLabels.end()) sourceLabels.push_back(label);
        }
        std::unordered_set<std::uint32_t> passages;
        for (const auto& component : network.components) {
            if (component.type != ExhaustComponentType::merge) continue;
            std::optional<std::size_t> row;
            auto faces = 0;
            auto oneRotor = true;
            for (const auto& mapping : network.cylinderConnections) {
                if (mapping.componentId != component.id) continue;
                const auto found = sourceRow.find(mapping.cylinderId);
                const auto rotor = rotorOf_ ? rotorOf_(mapping.cylinderId) : -1;
                if (found == sourceRow.end() || rotor < 0 || (row && *row != found->second)) oneRotor = false;
                else row = found->second;
                ++faces;
            }
            const auto fed = std::any_of(network.connections.begin(), network.connections.end(),
                [&component](const auto& connection) { return connection.toComponentId == component.id; });
            if (faces > 1 && oneRotor && !fed) passages.insert(component.id);
        }
        // A side branch (an H's balance pipe) meets its junction at right
        // angles: it stands in the junction's column, not downstream of it.
        // Counted as a step, it pushed one side's pipe two columns behind the
        // other's and crossed the drawing (owner report 2026-10-30).
        const auto lateral = [](const ExhaustComponentConnectionConfig& connection) {
            return connection.fromPort == exhaustSideBranchPort || connection.toPort == exhaustSideBranchPort;
        };
        std::unordered_map<std::uint32_t, int> levels;
        for (const auto& component : network.components) levels.emplace(component.id, 0);
        for (std::size_t pass = 0; pass < network.components.size(); ++pass) {
            bool changed = false;
            for (const auto& connection : network.connections) {
                const auto source = levels.find(connection.fromComponentId);
                const auto target = levels.find(connection.toComponentId);
                if (source == levels.end() || target == levels.end()) continue;
                const auto step = lateral(connection) || passages.contains(connection.fromComponentId) ? 0 : 1;
                const auto candidate = std::min(static_cast<int>(network.components.size()), source->second + step);
                if (candidate > target->second) {
                    target->second = candidate;
                    changed = true;
                }
            }
            if (!changed) break;
        }
        // Each node's row follows what feeds it (the mean row of its
        // cylinders and sources), so each bank's pipes keep to its side and a
        // balance pipe sits between the two it joins.
        std::unordered_map<std::uint32_t, double> rowKeys;
        const auto sourceCount = sourceLabels.size();
        for (std::size_t index = 0; index < network.components.size(); ++index) {
            const auto id = network.components[index].id;
            double sum = 0.0;
            int count = 0;
            for (const auto& mapping : network.cylinderConnections) {
                if (mapping.componentId != id) continue;
                const auto row = sourceRow.find(mapping.cylinderId);
                sum += static_cast<double>(row != sourceRow.end() ? row->second : sourceCount);
                ++count;
            }
            rowKeys[id] = count > 0 ? sum / count : static_cast<double>(sourceCount + index);
        }
        for (std::size_t pass = 0; pass < network.components.size(); ++pass) {
            bool changed = false;
            for (const auto& component : network.components) {
                if (std::any_of(network.cylinderConnections.begin(), network.cylinderConnections.end(),
                        [&](const auto& mapping) { return mapping.componentId == component.id; }))
                    continue;
                double sum = 0.0;
                int count = 0;
                // Through flow first; a node fed only sideways sits between
                // the junctions it joins.
                for (const auto sideways : { false, true }) {
                    for (const auto& connection : network.connections) {
                        if (lateral(connection) != sideways) continue;
                        const auto other = connection.toComponentId == component.id ? connection.fromComponentId
                            : sideways && connection.fromComponentId == component.id ? connection.toComponentId
                            : 0U;
                        if (other == 0U || other == component.id) continue;
                        sum += rowKeys[other];
                        ++count;
                    }
                    if (count > 0) break;
                }
                if (count == 0) continue;
                const auto key = sum / count;
                if (std::abs(key - rowKeys[component.id]) > 1.0e-9) {
                    rowKeys[component.id] = key;
                    changed = true;
                }
            }
            if (!changed) break;
        }

        int maximumLevel = 0;
        for (const auto& [id, level] : levels)
            if (!passages.contains(id)) maximumLevel = std::max(maximumLevel, level);
        std::vector<std::vector<const ExhaustComponentConfig*>> columns(
            static_cast<std::size_t>(maximumLevel + 1));
        for (const auto& component : network.components)
            if (!passages.contains(component.id))
                columns[static_cast<std::size_t>(levels[component.id])].push_back(&component);
        for (auto& column : columns)
            std::stable_sort(column.begin(), column.end(), [&rowKeys](const auto* a, const auto* b) {
                return rowKeys[a->id] < rowKeys[b->id];
            });

        const auto graph = getLocalBounds().toFloat().reduced(16.0F);
        const auto cylinderWidth = std::min(86.0F, graph.getWidth() * 0.14F);
        const auto nodesLeft = graph.getX() + cylinderWidth + 18.0F;
        const auto nodesWidth = std::max(100.0F, graph.getRight() - nodesLeft);
        const auto columnWidth = nodesWidth / static_cast<float>(std::max(1, maximumLevel + 1));
        const auto nodeWidth = std::clamp(columnWidth - 18.0F, 72.0F, 132.0F);
        constexpr float nodeHeight = 48.0F;

        for (std::size_t columnIndex = 0; columnIndex < columns.size(); ++columnIndex) {
            const auto& column = columns[columnIndex];
            const auto rowHeight = graph.getHeight() / static_cast<float>(std::max<std::size_t>(1U, column.size()));
            for (std::size_t row = 0; row < column.size(); ++row) {
                const auto x = nodesLeft + static_cast<float>(columnIndex) * columnWidth
                    + (columnWidth - nodeWidth) * 0.5F;
                const auto y = graph.getY() + static_cast<float>(row) * rowHeight
                    + (rowHeight - nodeHeight) * 0.5F;
                nodeBounds_.emplace(column[row]->id,
                    juce::Rectangle<float>(x, y, nodeWidth, nodeHeight));
            }
        }

        graphics.setColour(juce::Colour(0xff60716c));
        for (const auto& connection : network.connections) {
            const auto source = nodeBounds_.find(connection.fromComponentId);
            const auto target = nodeBounds_.find(connection.toComponentId);
            if (source == nodeBounds_.end() || target == nodeBounds_.end()) continue;
            juce::Path link;
            juce::Point<float> end;
            if (lateral(connection)) {
                // Edge to edge, across the flow.
                const auto down = target->second.getCentreY() >= source->second.getCentreY();
                const juce::Point<float> start { source->second.getCentreX(),
                                                 down ? source->second.getBottom() : source->second.getY() };
                end = { target->second.getCentreX(), down ? target->second.getY() : target->second.getBottom() };
                const auto controlOffset = std::max(10.0F, std::abs(end.y - start.y) * 0.38F) * (down ? 1.0F : -1.0F);
                link.startNewSubPath(start);
                link.cubicTo(start.x, start.y + controlOffset, end.x, end.y - controlOffset, end.x, end.y);
            } else {
                const juce::Point<float> start { source->second.getRight(), source->second.getCentreY() };
                end = { target->second.getX(), target->second.getCentreY() };
                link.startNewSubPath(start);
                const auto controlOffset = std::max(14.0F, std::abs(end.x - start.x) * 0.38F);
                link.cubicTo(start.x + controlOffset, start.y, end.x - controlOffset, end.y, end.x, end.y);
            }
            graphics.strokePath(link, juce::PathStrokeType(2.0F));
            graphics.fillEllipse(end.x - 3.5F, end.y - 3.5F, 7.0F, 7.0F);
        }

        const auto sourceRows = std::max<std::size_t>(1U, sourceCount);
        const auto sourceSpacing = graph.getHeight() / static_cast<float>(sourceRows);
        for (std::size_t index = 0; index < sourceCount; ++index) {
            const auto y = graph.getY() + (static_cast<float>(index) + 0.5F) * sourceSpacing;
            const juce::Rectangle<float> pill(graph.getX(), y - 11.0F, cylinderWidth, 22.0F);
            std::vector<std::uint32_t> pillCylinders;
            for (const auto cylinderId : path->cylinderIds)
                if (sourceRow[cylinderId] == index) pillCylinders.push_back(cylinderId);
            sourceBounds_.emplace_back(pill, std::move(pillCylinders));
            graphics.setColour(juce::Colour(0xff263532));
            graphics.fillRoundedRectangle(pill, 5.0F);
            graphics.setColour(juce::Colour(0xffa9bbb5));
            graphics.setFont(juce::FontOptions(11.0F, juce::Font::bold));
            graphics.drawText(sourceLabels[index], pill.toNearestInt(), juce::Justification::centred);
            // What this source feeds: its components, or past a rotor's port
            // passage, what the passage feeds.
            std::vector<std::uint32_t> targets;
            for (const auto& mapping : network.cylinderConnections) {
                const auto row = sourceRow.find(mapping.cylinderId);
                if (row == sourceRow.end() || row->second != index) continue;
                if (!passages.contains(mapping.componentId)) {
                    targets.push_back(mapping.componentId);
                    continue;
                }
                for (const auto& connection : network.connections)
                    if (connection.fromComponentId == mapping.componentId) targets.push_back(connection.toComponentId);
            }
            std::sort(targets.begin(), targets.end());
            targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
            graphics.setColour(juce::Colour(0xff60716c));
            for (const auto id : targets)
                if (const auto target = nodeBounds_.find(id); target != nodeBounds_.end())
                    graphics.drawLine(pill.getRight(), pill.getCentreY(),
                                      target->second.getX(), target->second.getCentreY(), 1.5F);
        }

        const auto selectedId = selectionProvider_ ? selectionProvider_() : 0U;
        for (const auto& component : network.components) {
            // A rotor's passage has no node: it is drawn inside its source.
            const auto placed = nodeBounds_.find(component.id);
            if (placed == nodeBounds_.end()) continue;
            auto bounds = placed->second;
            const auto selected = selectedId == component.id;
            const auto colour = componentTypeColour(component.type);
            graphics.setColour(colour.withAlpha(selected ? 0.92F : 0.62F));
            graphics.fillRoundedRectangle(bounds, 7.0F);
            graphics.setColour(selected ? juce::Colour(0xffffd086) : colour.brighter(0.5F));
            graphics.drawRoundedRectangle(bounds, 7.0F, selected ? 2.5F : 1.0F);
            graphics.setColour(juce::Colours::white);
            graphics.setFont(juce::FontOptions(12.0F, juce::Font::bold));
            graphics.drawText("#" + juce::String(component.id) + "  " + componentTypeName(component.type)
                                  + (component.inletValve ? "  + valve" : ""),
                              bounds.removeFromTop(25.0F).toNearestInt(), juce::Justification::centred);
            graphics.setColour(juce::Colour(0xffe1e9e6));
            graphics.setFont(juce::FontOptions(10.0F));
            const auto detail = juce::String(component.diameterMm, 0) + " mm"
                + (component.lengthMm > 0.0 ? "  x  " + juce::String(component.lengthMm, 0) + " mm" : "");
            graphics.drawText(detail, bounds.toNearestInt(), juce::Justification::centred);
        }

        // A connection being dragged: a line from where it started, the node
        // it would land on outlined.
        if (dragging_) {
            const auto target = hit(dragPoint_).component;
            if (target)
                if (const auto bounds = nodeBounds_.find(*target); bounds != nodeBounds_.end()) {
                    graphics.setColour(juce::Colour(0xffffd086));
                    graphics.drawRoundedRectangle(bounds->second.expanded(3.0F), 9.0F, 2.0F);
                }
            graphics.setColour(juce::Colour(0xffffd086));
            const float dashes[] { 6.0F, 4.0F };
            graphics.drawDashedLine({ press_.anchor, dragPoint_ }, dashes, 2, 2.0F);
        }
    }

    void mouseDown(const juce::MouseEvent& event) override {
        grabKeyboardFocus();
        dragging_ = false;
        const auto pressed = hit(event.position);
        if (event.mods.isPopupMenu()) {
            press_ = {};
            if (pressed.component) {
                if (selectionCallback_) selectionCallback_(*pressed.component);
                if (onComponentMenu) onComponentMenu(*pressed.component);
            } else if (!pressed.cylinders.empty() && onSourceMenu) {
                onSourceMenu(pressed.cylinders);
            }
            return;
        }
        press_ = pressed;
        if (pressed.component && selectionCallback_) selectionCallback_(*pressed.component);
    }

    void mouseDrag(const juce::MouseEvent& event) override {
        if (!press_.component && press_.cylinders.empty()) return;
        if (!dragging_ && event.getDistanceFromDragStart() < 6) return;
        dragging_ = true;
        dragPoint_ = event.position;
        repaint();
    }

    void mouseUp(const juce::MouseEvent& event) override {
        const auto pressed = std::exchange(press_, {});
        const auto dragged = std::exchange(dragging_, false);
        if (!dragged) return;
        repaint();
        const auto target = hit(event.position).component;
        if (!target) return;
        if (pressed.component) {
            if (*pressed.component != *target && onConnectComponents) onConnectComponents(*pressed.component, *target);
        } else if (onConnectCylinders) {
            onConnectCylinders(pressed.cylinders, *target);
        }
    }

private:
    /** What a point is on: a node, or a source's cylinders; and where a
     *  connection from it starts. */
    struct Hit {
        std::optional<std::uint32_t> component;
        std::vector<std::uint32_t> cylinders;
        juce::Point<float> anchor;
    };

    [[nodiscard]] Hit hit(juce::Point<float> point) const {
        for (const auto& [id, bounds] : nodeBounds_)
            if (bounds.contains(point)) return { id, {}, { bounds.getRight(), bounds.getCentreY() } };
        for (const auto& [bounds, cylinders] : sourceBounds_)
            if (bounds.contains(point)) return { std::nullopt, cylinders, { bounds.getRight(), bounds.getCentreY() } };
        return {};
    }

    PathProvider pathProvider_;
    SelectionProvider selectionProvider_;
    SelectionCallback selectionCallback_;
    RotorOf rotorOf_;
    std::unordered_map<std::uint32_t, juce::Rectangle<float>> nodeBounds_;
    std::vector<std::pair<juce::Rectangle<float>, std::vector<std::uint32_t>>> sourceBounds_;
    Hit press_;
    bool dragging_ { false };
    juce::Point<float> dragPoint_;
};

/** Where a menu of the designer opens: at the mouse. */
[[nodiscard]] juce::PopupMenu::Options menuAtMouse() {
    return juce::PopupMenu::Options {}.withMousePosition();
}

/** The fields the property editor writes, to tell an edit from a re-commit. */
[[nodiscard]] bool sameEditedFields(const ExhaustComponentConfig& a, const ExhaustComponentConfig& b) noexcept {
    return a.id == b.id && a.type == b.type && a.lengthMm == b.lengthMm && a.diameterMm == b.diameterMm
        && a.outletDiameterMm == b.outletDiameterMm && a.volumeLitres == b.volumeLitres
        && a.restriction == b.restriction && a.resonanceHz == b.resonanceHz
        && a.crossoverCoupling == b.crossoverCoupling && a.acousticGain == b.acousticGain
        && a.dischargeCoefficient == b.dischargeCoefficient
        && a.packingFlowResistivityPaSPerM2 == b.packingFlowResistivityPaSPerM2
        && a.packingThicknessMm == b.packingThicknessMm && a.perforatedOpenAreaRatio == b.perforatedOpenAreaRatio
        && a.catalystCellDensityCpsi == b.catalystCellDensityCpsi && a.catalystOpenAreaRatio == b.catalystOpenAreaRatio
        && a.catalystSubstrateVolumetricHeatCapacityJPerM3K == b.catalystSubstrateVolumetricHeatCapacityJPerM3K;
}

/** The From list's cylinders come after its components, from this item id. */
constexpr int cylinderItemBase = 10'000;
constexpr std::size_t maximumUndoSteps = 100;

} // namespace

class ExhaustDesignerWindow::DesignerContent final : public juce::Component {
public:
    DesignerContent(const EngineConfig& config, ApplyCallback applyCallback)
        : working_(config), applyCallback_(std::move(applyCallback)),
          componentModel_(
              [this] { return componentCount(); },
              [this](int row) { return componentText(row); },
              [this](int row) { selectComponentRow(row); }),
          connectionModel_(
              [this] { return connectionCount(); },
              [this](int row) { return connectionText(row); },
              [this](int row) { selectedConnectionRow_ = row; }),
          mappingModel_(
              [this] { return mappingCount(); },
              [this](int row) { return mappingText(row); },
              [this](int row) { selectMappingRow(row); }),
          graphCanvas_(
              [this] { return selectedPath(); },
              [this] { return selectedComponentId_; },
              [this](std::uint32_t id) { selectComponentId(id); },
              [this](std::uint32_t id) { return rotorOf(id); }) {
        setOpaque(true);
        setWantsKeyboardFocus(true);
        configureInterface();
        graphCanvas_.onConnectComponents = [this](std::uint32_t from, std::uint32_t to) { connectComponents(from, to); };
        graphCanvas_.onConnectCylinders = [this](const std::vector<std::uint32_t>& cylinders, std::uint32_t to) {
            connectCylinders(cylinders, to);
        };
        graphCanvas_.onComponentMenu = [this](std::uint32_t id) { showComponentMenu(id); };
        graphCanvas_.onSourceMenu = [this](const std::vector<std::uint32_t>& cylinders) { showSourceMenu(cylinders); };
        setConfig(config);
    }

    /** Ctrl+Z undoes the last edit of the working copy; Ctrl+Y or
     *  Ctrl+Shift+Z redoes it. A text field being typed in keeps its own. */
    bool keyPressed(const juce::KeyPress& key) override {
        if (!key.getModifiers().isCommandDown()) return false;
        const auto code = juce::CharacterFunctions::toUpperCase(static_cast<juce::juce_wchar>(key.getKeyCode()));
        if (code == 'Z' && !key.getModifiers().isShiftDown()) {
            undo();
            return true;
        }
        if (code == 'Y' || code == 'Z') {
            redo();
            return true;
        }
        return false;
    }

    void setConfig(const EngineConfig& config) {
        // An apply comes back here with the engine it applied: the edit
        // history is still this engine's. Any other engine starts afresh.
        if (!applying_) {
            undo_.clear();
            redo_.clear();
        }
        working_ = config;
        // The network the engine is actually running, kept so `apply()` can
        // report what the edit costs RELATIVE to it. A ratio against the user's
        // own baseline needs no calibrated threshold, which matters here: the
        // absolute substep rate that is fine for one exhaust is not meaningful
        // for another.
        baseline_ = config;
        ensurePathExists();
        selectedPathIndex_ = 0;
        selectedComponentId_ = 0;
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        rebuildPaths();
        selectFirstComponent();
        setStatus("Working copy loaded. No change has been applied yet.", false);
        rebuildAll();
        refreshUndoButtons();
    }

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour(0xff0d1312));
        const auto header = juce::Rectangle<int>(0, 0, getWidth(), 54);
        graphics.setGradientFill(juce::ColourGradient(juce::Colour(0xff1d2b27), 0.0F, 0.0F,
            juce::Colour(0xff111817), static_cast<float>(getWidth()), 0.0F, false));
        graphics.fillRect(header);
        graphics.setColour(juce::Colour(0xffdce7e3));
        graphics.setFont(juce::FontOptions(18.0F, juce::Font::bold));
        graphics.drawText("EXHAUST DESIGNER", 14, 0, 280, 54,
                          juce::Justification::centredLeft);
    }

    void resized() override {
        auto bounds = getLocalBounds().reduced(12);
        auto top = bounds.removeFromTop(42);
        top.removeFromLeft(290);
        pathSelector_.setBounds(top.removeFromLeft(185).reduced(2, 5));
        top.removeFromLeft(5);
        newPathButton_.setBounds(top.removeFromLeft(78).reduced(2, 5));
        top.removeFromLeft(5);
        deletePathButton_.setBounds(top.removeFromLeft(88).reduced(2, 5));
        top.removeFromLeft(5);
        legacyButton_.setBounds(top.removeFromLeft(170).reduced(2, 5));
        applyButton_.setBounds(top.removeFromRight(188).reduced(2, 5));

        auto pathTools = bounds.removeFromTop(34);
        pathTools.removeFromLeft(290);
        pathCylinderSelector_.setBounds(pathTools.removeFromLeft(220).reduced(2, 3));
        pathTools.removeFromLeft(6);
        destinationPathSelector_.setBounds(pathTools.removeFromLeft(220).reduced(2, 3));
        pathTools.removeFromLeft(6);
        moveCylinderButton_.setBounds(pathTools.removeFromLeft(130).reduced(2, 3));
        redoButton_.setBounds(pathTools.removeFromRight(80).reduced(2, 3));
        pathTools.removeFromRight(4);
        undoButton_.setBounds(pathTools.removeFromRight(80).reduced(2, 3));
        bounds.removeFromTop(5);

        statusLabel_.setBounds(bounds.removeFromBottom(29));
        bounds.removeFromBottom(8);

        auto graphArea = bounds.removeFromTop(std::clamp(bounds.getHeight() * 46 / 100, 260, 390));
        graphGroup_.setBounds(graphArea);
        graphCanvas_.setBounds(graphArea.reduced(10, 24));
        // On the flow view's title line, at its right: what it draws on.
        auto crossoverTools = juce::Rectangle<int>(graphArea.getRight() - 10 - 2 * 128 - 6, graphArea.getY(), 2 * 128 + 6, 22);
        xPipeButton_.setBounds(crossoverTools.removeFromRight(128));
        crossoverTools.removeFromRight(6);
        hPipeButton_.setBounds(crossoverTools);
        bounds.removeFromTop(8);

        auto componentArea = bounds.removeFromLeft(std::clamp(bounds.getWidth() * 23 / 100, 240, 330));
        bounds.removeFromLeft(8);
        auto propertyArea = bounds.removeFromLeft(std::clamp(bounds.getWidth() * 42 / 100, 360, 480));
        bounds.removeFromLeft(8);
        auto topologyArea = bounds;

        componentGroup_.setBounds(componentArea);
        auto componentInner = componentArea.reduced(10, 24);
        auto componentActions = componentInner.removeFromBottom(76);
        componentList_.setBounds(componentInner);
        addTypeSelector_.setBounds(componentActions.removeFromTop(32));
        componentActions.removeFromTop(5);
        addButton_.setBounds(componentActions.removeFromLeft((componentActions.getWidth() - 6) / 2));
        componentActions.removeFromLeft(6);
        deleteComponentButton_.setBounds(componentActions);

        propertyGroup_.setBounds(propertyArea);
        auto propertyInner = propertyArea.reduced(10, 25);
        auto typeRow = propertyInner.removeFromTop(30);
        componentTypeLabel_.setBounds(typeRow.removeFromLeft(92));
        componentTypeSelector_.setBounds(typeRow);
        propertyInner.removeFromTop(7);
        const auto columnGap = 8;
        const auto columnWidth = (propertyInner.getWidth() - columnGap) / 2;
        auto leftColumn = propertyInner.removeFromLeft(columnWidth);
        propertyInner.removeFromLeft(columnGap);
        auto rightColumn = propertyInner;
        layoutPropertyColumn(leftColumn, { 0, 1, 2, 3, 4, 5 });
        layoutPropertyColumn(rightColumn, { 6, 7, 8, 9, 10, 11 });
        auto propertyActions = juce::Rectangle<int>(
            propertyArea.getX() + 12, propertyArea.getBottom() - 40,
            propertyArea.getWidth() - 24, 28);
        const auto actionWidth = (propertyActions.getWidth() - 12) / 3;
        packingDemoButton_.setBounds(propertyActions.removeFromLeft(actionWidth));
        propertyActions.removeFromLeft(6);
        packingBypassButton_.setBounds(propertyActions.removeFromLeft(actionWidth));
        propertyActions.removeFromLeft(6);
        updateComponentButton_.setBounds(propertyActions);

        topologyGroup_.setBounds(topologyArea);
        auto topologyInner = topologyArea.reduced(10, 24);
        auto connectionArea = topologyInner.removeFromTop(topologyInner.getHeight() / 2);
        connectionTitle_.setBounds(connectionArea.removeFromTop(22));
        auto connectionEditors = connectionArea.removeFromTop(30);
        fromSelector_.setBounds(connectionEditors.removeFromLeft((connectionEditors.getWidth() - 6) / 2));
        connectionEditors.removeFromLeft(6);
        toSelector_.setBounds(connectionEditors);
        connectionArea.removeFromTop(5);
        auto connectionButtons = connectionArea.removeFromBottom(28);
        addConnectionButton_.setBounds(connectionButtons.removeFromLeft((connectionButtons.getWidth() - 6) / 2));
        connectionButtons.removeFromLeft(6);
        deleteConnectionButton_.setBounds(connectionButtons);
        connectionList_.setBounds(connectionArea);

        topologyInner.removeFromTop(7);
        mappingTitle_.setBounds(topologyInner.removeFromTop(22));
        auto mappingEditors = topologyInner.removeFromTop(30);
        cylinderSelector_.setBounds(mappingEditors.removeFromLeft((mappingEditors.getWidth() - 6) / 2));
        mappingEditors.removeFromLeft(6);
        mappingComponentSelector_.setBounds(mappingEditors);
        topologyInner.removeFromTop(5);
        auto mappingButtons = topologyInner.removeFromBottom(28);
        assignCylinderButton_.setBounds(mappingButtons.removeFromLeft((mappingButtons.getWidth() - 6) / 2));
        mappingButtons.removeFromLeft(6);
        deleteMappingButton_.setBounds(mappingButtons);
        mappingList_.setBounds(topologyInner);
    }

private:
    void configureInterface() {
        graphGroup_.setText("Flow view (cylinders to outlets)");
        componentGroup_.setText("Components");
        propertyGroup_.setText("Component properties");
        topologyGroup_.setText("Connections and assignments");
        for (auto* group : std::array<juce::GroupComponent*, 4> {
                 &graphGroup_, &componentGroup_, &propertyGroup_, &topologyGroup_ }) {
            group->setColour(juce::GroupComponent::outlineColourId, juce::Colour(0xff31413d));
            group->setColour(juce::GroupComponent::textColourId, juce::Colour(0xff9eb3ac));
            addAndMakeVisible(*group);
        }

        pathSelector_.setTextWhenNothingSelected("Exhaust path");
        pathSelector_.onChange = [this] {
            const auto nextPathIndex = std::max(0, pathSelector_.getSelectedItemIndex());
            if (nextPathIndex != selectedPathIndex_ && !commitSelectedComponentEdits()) {
                pathSelector_.setSelectedItemIndex(selectedPathIndex_, juce::dontSendNotification);
                return;
            }
            selectedPathIndex_ = nextPathIndex;
            selectedComponentId_ = 0;
            selectedConnectionRow_ = -1;
            selectedMappingRow_ = -1;
            selectFirstComponent();
            rebuildAll();
        };
        newPathButton_.setButtonText("+ PATH");
        newPathButton_.setTooltip(
            "Creates a path and moves the selected cylinder into it, so no path is ever empty.");
        newPathButton_.onClick = [this] { createPath(); };
        deletePathButton_.setButtonText("- PATH");
        deletePathButton_.setTooltip(
            "Deletes the current path; its cylinders join the destination, either with a Y junction "
            "(both collectors kept) or rebuilt as one N-into-1 collector.");
        deletePathButton_.onClick = [this] { deletePath(); };
        undoButton_.setButtonText("UNDO");
        undoButton_.setTooltip("Undoes the last edit of the working copy (Ctrl+Z).");
        undoButton_.setComponentID("exhaust-undo");
        undoButton_.onClick = [this] { undo(); };
        redoButton_.setButtonText("REDO");
        redoButton_.setTooltip("Redoes the edit just undone (Ctrl+Y or Ctrl+Shift+Z).");
        redoButton_.setComponentID("exhaust-redo");
        redoButton_.onClick = [this] { redo(); };
        applyButton_.setComponentID("exhaust-apply");
        legacyButton_.setButtonText("GENERATE FROM LEGACY");
        legacyButton_.setTooltip("Replaces the selected path's graph with a simple, valid network.");
        legacyButton_.onClick = [this] { generateLegacyNetwork(); };
        hPipeButton_.setButtonText("+ H-PIPE");
        hPipeButton_.setTooltip("Joins the two pipes (the banks') with a balance pipe across, in one click: "
                                "a third of the way from the collectors, 250 mm, the pipes' bore. A dual exhaust "
                                "drawn as two paths becomes one path first, both exhausts kept. Undo takes it off.");
        hPipeButton_.setComponentID("exhaust-add-h-pipe");
        hPipeButton_.onClick = [this] { addCrossover(ExhaustCrossoverKind::hPipe); };
        xPipeButton_.setButtonText("+ X-PIPE");
        xPipeButton_.setTooltip("Crosses the two pipes (the banks') in an X, in one click: a third of the way "
                                "from the collectors, 30 % of each pipe's pulse into the other. A dual exhaust "
                                "drawn as two paths becomes one path first, both exhausts kept. Undo takes it off.");
        xPipeButton_.setComponentID("exhaust-add-x-pipe");
        xPipeButton_.onClick = [this] { addCrossover(ExhaustCrossoverKind::xPipe); };
        applyButton_.setButtonText("VALIDATE AND APPLY");
        applyButton_.setColour(juce::TextButton::buttonColourId, juce::Colour(0xff477d68));
        applyButton_.onClick = [this] { apply(); };
        pathCylinderSelector_.setTextWhenNothingSelected("Cylinder to move");
        destinationPathSelector_.setTextWhenNothingSelected("Destination path");
        moveCylinderButton_.setButtonText("MOVE");
        moveCylinderButton_.setTooltip(
            "Moves the cylinder to the chosen path and repairs the roots of the custom DAG.");
        moveCylinderButton_.onClick = [this] { moveCylinderToPath(); };

        componentList_.setModel(&componentModel_);
        componentList_.setRowHeight(26);
        connectionList_.setModel(&connectionModel_);
        connectionList_.setRowHeight(23);
        mappingList_.setModel(&mappingModel_);
        mappingList_.setRowHeight(23);
        for (auto* list : std::array<juce::ListBox*, 3> {
                 &componentList_, &connectionList_, &mappingList_ }) {
            list->setColour(juce::ListBox::backgroundColourId, juce::Colour(0xff111817));
            list->setColour(juce::ListBox::outlineColourId, juce::Colour(0xff293633));
            list->setOutlineThickness(1);
            addAndMakeVisible(*list);
        }

        populateTypeSelector(addTypeSelector_);
        addTypeSelector_.setSelectedItemIndex(0, juce::dontSendNotification);
        addButton_.setButtonText("ADD");
        addButton_.onClick = [this] { addComponent(); };
        deleteComponentButton_.setButtonText("DELETE");
        deleteComponentButton_.onClick = [this] { deleteComponent(); };
        componentList_.setComponentID("exhaust-components");
        addTypeSelector_.setComponentID("exhaust-add-type");
        addButton_.setComponentID("exhaust-add");
        deleteComponentButton_.setComponentID("exhaust-delete-component");

        componentTypeLabel_.setText("Type", juce::dontSendNotification);
        componentTypeLabel_.setColour(juce::Label::textColourId, juce::Colour(0xffaebbb7));
        populateTypeSelector(componentTypeSelector_);
        const std::array<const char*, 12> names {
            "ID", "Length (mm)", "Inlet diameter (mm)",
            "Outlet diameter (mm, 0 = constant)", "Volume (L)",
            "Flow restriction (K)", "Branch tuning (Hz, 0 = length)",
            "Legacy fallback gain", "Outlet Cd",
            "Packing resistivity (Pa.s/m2)", "Packing thickness (mm)",
            "Perforate open area (0..1)"
        };
        for (std::size_t index = 0; index < propertyLabels_.size(); ++index) {
            propertyLabels_[index].setText(names[index], juce::dontSendNotification);
            propertyLabels_[index].setColour(juce::Label::textColourId, juce::Colour(0xffaebbb7));
            propertyLabels_[index].setFont(juce::FontOptions(11.5F));
            propertyEditors_[index].setColour(juce::TextEditor::backgroundColourId,
                                               juce::Colour(0xff17211f));
            propertyEditors_[index].setColour(juce::TextEditor::outlineColourId,
                                               juce::Colour(0xff31413d));
            propertyEditors_[index].onReturnKey = [this] { updateSelectedComponent(); };
            addAndMakeVisible(propertyLabels_[index]);
            addAndMakeVisible(propertyEditors_[index]);
        }
        propertyEditors_[4].setTooltip(
            "On a terminal resonator, this volume is the closed cavity at the end of the branch. "
            "On a packed muffler, it is the gross shell volume around the core; "
            "it must exceed the volume swept by the core. Zero gives a rigid "
            "quarter-wave termination on a resonator.");
        propertyEditors_[2].setTooltip(
            "Packed muffler: diameter of the perforated core that carries the flow. "
            "Dry muffler: diameter of the reactive chamber.");
        propertyEditors_[5].setTooltip(
            "Pressure-loss coefficient of the mean flow. Its physical acoustic effect is "
            "indirect, through the pressure and flow computed by the gas solver.");
        propertyEditors_[6].setTooltip(
            "Used by the physical network only for a terminal resonator: the reference "
            "acoustic length is c/(4f). On an X-pipe, k is the crossed power "
            "amplitude and sqrt(1-k^2) stays in the paired pipe.");
        propertyEditors_[7].setTooltip(
            "Compatibility with the fallback audio render. The physical waveguide stays passive and "
            "does not apply this arbitrary gain.");
        propertyEditors_[9].setTooltip(
            "Muffler: packing resistivity. Catalyst: cell density per square inch.");
        propertyEditors_[10].setTooltip(
            "Muffler: packing thickness. Catalyst: fraction of the face that is actually open.");
        propertyEditors_[11].setTooltip(
            "Muffler: open area of the perforated tube. Catalyst: effective volumetric heat capacity of the solid substrate.");
        updateComponentButton_.setButtonText("UPDATE COMPONENT");
        updateComponentButton_.onClick = [this] { updateSelectedComponent(); };
        packingDemoButton_.setButtonText("DEMO PACKING");
        packingDemoButton_.onClick = [this] { setPackingPreset(true); };
        packingBypassButton_.setButtonText("BYPASS PACKING");
        packingBypassButton_.onClick = [this] { setPackingPreset(false); };
        componentTypeSelector_.onChange = [this] {
            updatePackingEditorAvailability(true);
        };

        connectionTitle_.setText("Component -> component connections", juce::dontSendNotification);
        mappingTitle_.setText("Cylinder -> first component", juce::dontSendNotification);
        for (auto* label : std::array<juce::Label*, 3> {
                 &componentTypeLabel_, &connectionTitle_, &mappingTitle_ }) {
            label->setColour(juce::Label::textColourId, juce::Colour(0xffb6c5c0));
            label->setFont(juce::FontOptions(12.0F, juce::Font::bold));
            addAndMakeVisible(*label);
        }
        fromSelector_.setTextWhenNothingSelected("From (component or cylinder)");
        fromSelector_.setTooltip("A component, or a cylinder: a cylinder can feed any part. "
                                 "On the flow view, drag from a cylinder or a part onto another part.");
        toSelector_.setTextWhenNothingSelected("To");
        fromSelector_.setComponentID("exhaust-from");
        toSelector_.setComponentID("exhaust-to");
        addConnectionButton_.setComponentID("exhaust-connect");
        graphCanvas_.setTooltip("Drag from a cylinder or a part onto another part to connect them; "
                                "right-click a part to replace or delete it, a cylinder to disconnect it.");
        cylinderSelector_.setTextWhenNothingSelected("Cylinder");
        mappingComponentSelector_.setTextWhenNothingSelected("Component");
        addConnectionButton_.setButtonText("CONNECT");
        addConnectionButton_.onClick = [this] { addConnection(); };
        deleteConnectionButton_.setButtonText("DELETE");
        deleteConnectionButton_.onClick = [this] { deleteConnection(); };
        assignCylinderButton_.setButtonText("ASSIGN");
        assignCylinderButton_.onClick = [this] { assignCylinder(); };
        deleteMappingButton_.setButtonText("UNASSIGN");
        deleteMappingButton_.onClick = [this] { deleteMapping(); };

        statusLabel_.setFont(juce::FontOptions(12.0F));
        statusLabel_.setJustificationType(juce::Justification::centredLeft);
        addAndMakeVisible(pathSelector_);
        addAndMakeVisible(newPathButton_);
        addAndMakeVisible(deletePathButton_);
        addAndMakeVisible(legacyButton_);
        addAndMakeVisible(hPipeButton_);
        addAndMakeVisible(xPipeButton_);
        addAndMakeVisible(applyButton_);
        addAndMakeVisible(pathCylinderSelector_);
        addAndMakeVisible(destinationPathSelector_);
        addAndMakeVisible(moveCylinderButton_);
        addAndMakeVisible(undoButton_);
        addAndMakeVisible(redoButton_);
        addAndMakeVisible(graphCanvas_);
        addAndMakeVisible(addTypeSelector_);
        addAndMakeVisible(addButton_);
        addAndMakeVisible(deleteComponentButton_);
        addAndMakeVisible(componentTypeSelector_);
        addAndMakeVisible(updateComponentButton_);
        addAndMakeVisible(packingDemoButton_);
        addAndMakeVisible(packingBypassButton_);
        addAndMakeVisible(fromSelector_);
        addAndMakeVisible(toSelector_);
        addAndMakeVisible(addConnectionButton_);
        addAndMakeVisible(deleteConnectionButton_);
        addAndMakeVisible(cylinderSelector_);
        addAndMakeVisible(mappingComponentSelector_);
        addAndMakeVisible(assignCylinderButton_);
        addAndMakeVisible(deleteMappingButton_);
        addAndMakeVisible(statusLabel_);
    }

    static void populateTypeSelector(juce::ComboBox& selector) {
        selector.clear(juce::dontSendNotification);
        for (int type = static_cast<int>(ExhaustComponentType::pipe);
             type <= static_cast<int>(ExhaustComponentType::crossover); ++type) {
            selector.addItem(componentTypeName(static_cast<ExhaustComponentType>(type)), type + 1);
        }
    }

    void ensurePathExists() {
        if (!working_.exhaustPaths.empty()) return;
        ExhaustPathConfig path;
        path.id = 1;
        path.geometry = working_.exhaust;
        path.cylinderIds.reserve(working_.cylinders.size());
        for (const auto& cylinder : working_.cylinders) path.cylinderIds.push_back(cylinder.id);
        working_.exhaustPaths.push_back(std::move(path));
    }

    [[nodiscard]] ExhaustPathConfig* selectedPath() noexcept {
        if (selectedPathIndex_ < 0
            || selectedPathIndex_ >= static_cast<int>(working_.exhaustPaths.size())) return nullptr;
        return &working_.exhaustPaths[static_cast<std::size_t>(selectedPathIndex_)];
    }

    [[nodiscard]] const ExhaustPathConfig* selectedPath() const noexcept {
        if (selectedPathIndex_ < 0
            || selectedPathIndex_ >= static_cast<int>(working_.exhaustPaths.size())) return nullptr;
        return &working_.exhaustPaths[static_cast<std::size_t>(selectedPathIndex_)];
    }

    [[nodiscard]] ExhaustNetworkConfig* selectedNetwork() noexcept {
        auto* path = selectedPath();
        return path != nullptr && path->network ? &*path->network : nullptr;
    }

    [[nodiscard]] const ExhaustNetworkConfig* selectedNetwork() const noexcept {
        const auto* path = selectedPath();
        return path != nullptr && path->network ? &*path->network : nullptr;
    }

    [[nodiscard]] ExhaustComponentConfig* selectedComponent() noexcept {
        auto* network = selectedNetwork();
        if (network == nullptr) return nullptr;
        const auto found = std::find_if(network->components.begin(), network->components.end(),
            [this](const auto& component) { return component.id == selectedComponentId_; });
        return found != network->components.end() ? &*found : nullptr;
    }

    [[nodiscard]] const ExhaustComponentConfig* componentAt(int row) const noexcept {
        const auto* network = selectedNetwork();
        if (network == nullptr || row < 0 || row >= static_cast<int>(network->components.size())) return nullptr;
        return &network->components[static_cast<std::size_t>(row)];
    }

    void rebuildPaths() {
        pathSelector_.clear(juce::dontSendNotification);
        for (std::size_t index = 0; index < working_.exhaustPaths.size(); ++index) {
            const auto& path = working_.exhaustPaths[index];
            pathSelector_.addItem("Path #" + juce::String(path.id) + "  ("
                + juce::String(path.cylinderIds.size()) + " cyl.)", static_cast<int>(index) + 1);
        }
        pathSelector_.setSelectedItemIndex(selectedPathIndex_, juce::dontSendNotification);
    }

    void rebuildAll() {
        rebuildPathManagementSelectors();
        rebuildComponentSelectors();
        rebuildCylinderSelector();
        componentList_.updateContent();
        connectionList_.updateContent();
        mappingList_.updateContent();
        loadSelectedComponent();
        graphCanvas_.repaint();
        resized();
    }

    [[nodiscard]] std::optional<std::uint32_t> selectedPathCylinderId() const {
        const auto* path = selectedPath();
        const auto index = pathCylinderSelector_.getSelectedItemIndex();
        if (path == nullptr || index < 0 || index >= static_cast<int>(path->cylinderIds.size()))
            return std::nullopt;
        return path->cylinderIds[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] std::optional<std::uint32_t> selectedDestinationPathId() const {
        const auto itemId = destinationPathSelector_.getSelectedId();
        if (itemId <= 0) return std::nullopt;
        const auto index = static_cast<std::size_t>(itemId - 1);
        if (index >= working_.exhaustPaths.size()) return std::nullopt;
        return working_.exhaustPaths[index].id;
    }

    void rebuildPathManagementSelectors() {
        const auto previousCylinder = selectedPathCylinderId();
        const auto previousDestination = selectedDestinationPathId();
        pathCylinderSelector_.clear(juce::dontSendNotification);
        destinationPathSelector_.clear(juce::dontSendNotification);
        const auto* path = selectedPath();
        if (path != nullptr) {
            for (std::size_t index = 0; index < path->cylinderIds.size(); ++index) {
                const auto cylinderId = path->cylinderIds[index];
                pathCylinderSelector_.addItem(cylinderName(cylinderId),
                                               static_cast<int>(index) + 1);
            }
            const auto preserve = previousCylinder
                ? std::find(path->cylinderIds.begin(), path->cylinderIds.end(), *previousCylinder)
                : path->cylinderIds.end();
            if (preserve != path->cylinderIds.end()) {
                pathCylinderSelector_.setSelectedItemIndex(
                    static_cast<int>(std::distance(path->cylinderIds.begin(), preserve)),
                    juce::dontSendNotification);
            } else if (!path->cylinderIds.empty()) {
                pathCylinderSelector_.setSelectedItemIndex(0, juce::dontSendNotification);
            }
        }

        int preferredDestinationRow = -1;
        int row = 0;
        for (std::size_t index = 0; index < working_.exhaustPaths.size(); ++index) {
            if (static_cast<int>(index) == selectedPathIndex_) continue;
            const auto& candidate = working_.exhaustPaths[index];
            destinationPathSelector_.addItem("To path #" + juce::String(candidate.id)
                + " (" + juce::String(candidate.cylinderIds.size()) + " cyl.)",
                static_cast<int>(index) + 1);
            if (previousDestination && candidate.id == *previousDestination)
                preferredDestinationRow = row;
            ++row;
        }
        if (destinationPathSelector_.getNumItems() > 0) {
            destinationPathSelector_.setSelectedItemIndex(
                preferredDestinationRow >= 0 ? preferredDestinationRow : 0,
                juce::dontSendNotification);
        }
        const auto canLeaveSourceNonEmpty = path != nullptr && path->cylinderIds.size() > 1U;
        newPathButton_.setEnabled(canLeaveSourceNonEmpty && working_.exhaustPaths.size() < 8U);
        moveCylinderButton_.setEnabled(canLeaveSourceNonEmpty
            && destinationPathSelector_.getNumItems() > 0);
        deletePathButton_.setEnabled(working_.exhaustPaths.size() > 1U);
        destinationPathSelector_.setEnabled(working_.exhaustPaths.size() > 1U);
    }

    void selectEditedPath(std::uint32_t pathId) {
        const auto found = std::find_if(working_.exhaustPaths.begin(), working_.exhaustPaths.end(),
            [pathId](const ExhaustPathConfig& path) { return path.id == pathId; });
        selectedPathIndex_ = found != working_.exhaustPaths.end()
            ? static_cast<int>(std::distance(working_.exhaustPaths.begin(), found)) : 0;
        selectedComponentId_ = 0;
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        rebuildPaths();
        selectFirstComponent();
        rebuildAll();
    }

    [[nodiscard]] bool commitSelectedComponentEdits() {
        if (selectedComponent() == nullptr) return true;
        updateSelectedComponent();
        return !statusIsError_;
    }

    void createPath() {
        if (!commitSelectedComponentEdits()) return;
        const auto* source = selectedPath();
        const auto cylinderId = selectedPathCylinderId();
        if (source == nullptr || !cylinderId) {
            setStatus("Select a cylinder of the current path.", true);
            return;
        }
        auto before = snapshot();
        const auto result = ExhaustPathTopologyEditor::createPathFromCylinder(
            working_, source->id, *cylinderId);
        if (!result) {
            setStatus("Creation rejected: " + utf8(result.error), true);
            return;
        }
        pushUndo(std::move(before));
        selectEditedPath(result.selectedPathId);
        setStatus("New path created; the cylinder was moved atomically.", false);
    }

    void moveCylinderToPath() {
        if (!commitSelectedComponentEdits()) return;
        const auto* source = selectedPath();
        const auto cylinderId = selectedPathCylinderId();
        const auto destinationId = selectedDestinationPathId();
        if (source == nullptr || !cylinderId || !destinationId) {
            setStatus("Select the cylinder and the destination path.", true);
            return;
        }
        auto before = snapshot();
        const auto result = ExhaustPathTopologyEditor::moveCylinder(
            working_, *cylinderId, source->id, *destinationId);
        if (!result) {
            setStatus("Move rejected: " + utf8(result.error), true);
            return;
        }
        pushUndo(std::move(before));
        selectEditedPath(result.selectedPathId);
        setStatus("Cylinder moved: it joins its neighbours' collector with a primary like theirs.", false);
    }

    /** Asks how the path's cylinders join the destination, then removes it. */
    void deletePath() {
        if (!commitSelectedComponentEdits()) return;
        const auto* source = selectedPath();
        const auto destinationId = selectedDestinationPathId();
        if (source == nullptr || !destinationId) {
            setStatus("Choose the path that will receive the cylinders before deleting.", true);
            return;
        }
        const auto pathId = source->id;
        juce::PopupMenu menu;
        menu.addSectionHeader("Delete path #" + juce::String(pathId) + " into path #" + juce::String(*destinationId));
        menu.addItem(1, "Join with a Y junction (both collectors kept)");
        menu.addItem(2, "Rebuild as one N-into-1 collector");
        menu.showMenuAsync(juce::PopupMenu::Options {}.withTargetComponent(&deletePathButton_),
            [safe = juce::Component::SafePointer<DesignerContent>(this), pathId, destination = *destinationId](int choice) {
                if (safe == nullptr || choice <= 0) return;
                safe->removePathInto(pathId, destination,
                                     choice == 1 ? ExhaustPathJoin::yJunction : ExhaustPathJoin::rebuild);
            });
    }

    void removePathInto(std::uint32_t pathId, std::uint32_t destinationId, ExhaustPathJoin join) {
        auto before = snapshot();
        const auto result = ExhaustPathTopologyEditor::removePath(working_, pathId, destinationId, join);
        if (!result) {
            setStatus("Deletion rejected: " + utf8(result.error), true);
            return;
        }
        pushUndo(std::move(before));
        selectEditedPath(result.selectedPathId);
        if (!result.note.empty())
            setStatus("Path deleted. " + utf8(result.note), false);
        else if (join == ExhaustPathJoin::yJunction)
            setStatus("Path deleted: both collectors meet in a Y; the pipe after it gave the legs their length.", false);
        else
            setStatus("Path deleted: every cylinder has its own primary into one collector.", false);
    }

    /** An H-pipe or an X-pipe between the banks' pipes, in one step. */
    void addCrossover(ExhaustCrossoverKind kind) {
        if (!commitSelectedComponentEdits()) return;
        const auto* path = selectedPath();
        if (path == nullptr) return;
        auto before = snapshot();
        const auto result = ExhaustPathTopologyEditor::addCrossover(working_, path->id, kind,
                                                                    selectedDestinationPathId());
        if (!result) {
            setStatus(utf8(result.error), true);
            return;
        }
        pushUndo(std::move(before));
        selectEditedPath(result.selectedPathId);
        selectComponentId(result.componentId);
        const auto joined = result.joinedPathId != 0U
            ? " Path #" + juce::String(result.joinedPathId) + " joined this one, both exhausts kept." : juce::String();
        setStatus((kind == ExhaustCrossoverKind::hPipe
                      ? "H-pipe fitted: balance pipe #" + juce::String(result.componentId)
                            + " across the two pipes; set its length and bore here."
                      : "X-pipe fitted: #" + juce::String(result.componentId)
                            + " crosses the two pipes; set its cross-coupling k here.")
                      + joined + " Validate and apply to hear it.",
                  false);
    }

    /** The working copy before an edit, for undo. */
    struct Snapshot {
        EngineConfig config;
        int pathIndex {};
        std::uint32_t componentId {};
    };

    [[nodiscard]] Snapshot snapshot() const { return { working_, selectedPathIndex_, selectedComponentId_ }; }

    void pushUndo(Snapshot before) {
        undo_.push_back(std::move(before));
        if (undo_.size() > maximumUndoSteps) undo_.erase(undo_.begin());
        redo_.clear();
        refreshUndoButtons();
    }

    void rememberForUndo() { pushUndo(snapshot()); }

    void refreshUndoButtons() {
        undoButton_.setEnabled(!undo_.empty());
        redoButton_.setEnabled(!redo_.empty());
    }

    /** The working copy goes back to `from`'s last state; what it was goes on `to`. */
    void restore(std::vector<Snapshot>& from, std::vector<Snapshot>& to, const juce::String& done) {
        if (from.empty()) return;
        to.push_back(snapshot());
        auto state = std::move(from.back());
        from.pop_back();
        working_ = std::move(state.config);
        selectedPathIndex_ = std::clamp(state.pathIndex, 0, std::max(0, static_cast<int>(working_.exhaustPaths.size()) - 1));
        selectedComponentId_ = state.componentId;
        if (selectedComponent() == nullptr) selectFirstComponent();
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        rebuildPaths();
        rebuildAll();
        refreshUndoButtons();
        setStatus(done + " (" + juce::String(static_cast<int>(undo_.size())) + " to undo, "
                      + juce::String(static_cast<int>(redo_.size())) + " to redo).", false);
    }

    void undo() { restore(undo_, redo_, "Undone"); }
    void redo() { restore(redo_, undo_, "Redone"); }

    /** Applies `edit` to a copy of the selected path's graph; the working copy
     *  takes it, as one undo step, only if it returns no error. */
    template <typename Edit>
    [[nodiscard]] bool editNetwork(Edit&& edit) {
        auto* network = selectedNetwork();
        if (network == nullptr) return false;
        auto edited = *network;
        if (const std::string error = edit(edited); !error.empty()) {
            setStatus(utf8(error), true);
            return false;
        }
        rememberForUndo();
        *network = std::move(edited);
        return true;
    }

    [[nodiscard]] const ExhaustComponentConfig* findComponent(std::uint32_t id) const {
        const auto* network = selectedNetwork();
        if (network == nullptr) return nullptr;
        const auto found = std::find_if(network->components.begin(), network->components.end(),
            [id](const ExhaustComponentConfig& component) { return component.id == id; });
        return found != network->components.end() ? &*found : nullptr;
    }

    void connectComponents(std::uint32_t from, std::uint32_t to) {
        if (!commitSelectedComponentEdits()) return;
        if (!editNetwork([from, to](ExhaustNetworkConfig& network) {
                return ExhaustNetworkEditor::connectComponents(network, from, to);
            }))
            return;
        selectedConnectionRow_ = connectionCount() - 1;
        setStatus("Connected #" + juce::String(from) + " -> #" + juce::String(to)
                      + ". Validation will catch cycles and invalid cardinalities.", false);
        rebuildAll();
        connectionList_.selectRow(selectedConnectionRow_);
    }

    /** A cylinder, or a rotor's faces, now feed `to`. */
    void connectCylinders(const std::vector<std::uint32_t>& cylinders, std::uint32_t to) {
        if (cylinders.empty() || !commitSelectedComponentEdits()) return;
        const auto* target = findComponent(to);
        if (target == nullptr) return;
        if (cylinders.size() > 1U && target->type != ExhaustComponentType::merge) {
            setStatus("A rotor's three faces open onto one port: drop the rotor on a collector (its port passage).", true);
            return;
        }
        if (!editNetwork([&cylinders, to](ExhaustNetworkConfig& network) {
                for (const auto cylinderId : cylinders)
                    if (auto error = ExhaustNetworkEditor::connectCylinder(network, cylinderId, to); !error.empty())
                        return error;
                return std::string {};
            }))
            return;
        selectedMappingRow_ = -1;
        const auto rotor = rotorOf(cylinders.front());
        const auto source = rotor >= 0 ? "Rotor " + juce::String(rotor + 1) : cylinderName(cylinders.front());
        setStatus(source + " now feeds #" + juce::String(to) + ".", false);
        rebuildAll();
    }

    void showComponentMenu(std::uint32_t id) {
        const auto* component = findComponent(id);
        if (component == nullptr) return;
        juce::PopupMenu replace;
        for (int type = static_cast<int>(ExhaustComponentType::pipe);
             type <= static_cast<int>(ExhaustComponentType::crossover); ++type)
            replace.addItem(type + 1, componentTypeName(static_cast<ExhaustComponentType>(type)),
                            type != static_cast<int>(component->type));
        juce::PopupMenu menu;
        menu.addSectionHeader("#" + juce::String(id) + "  " + componentTypeName(component->type));
        menu.addSubMenu("Replace with", replace);
        const auto bypassable = component->type != ExhaustComponentType::merge
            && component->type != ExhaustComponentType::splitter
            && component->type != ExhaustComponentType::crossover
            && component->type != ExhaustComponentType::outlet;
        // A two-stroke's expansion chamber takes no active valve (owner,
        // 2026-11-14); one already there (an old file) can still come off.
        const auto twoStroke = working_.cycle == EngineCycle::twoStroke;
        if (!twoStroke) menu.addItem(101, "Bypass it with an active valve", bypassable);
        if (component->type == ExhaustComponentType::pipe && (!twoStroke || component->inletValve))
            menu.addItem(102, "Active valve at its inlet", true, component->inletValve);
        if (exhaustValveFitted(working_)) menu.addItem(103, "Valve actuator settings...");
        menu.addItem(100, "Delete");
        menu.showMenuAsync(menuAtMouse(), [safe = juce::Component::SafePointer<DesignerContent>(this), id](int choice) {
            if (safe != nullptr) safe->componentMenuChosen(id, choice);
        });
    }

    void componentMenuChosen(std::uint32_t id, int choice) {
        if (choice == 100) {
            if (trySelectComponent(id)) deleteComponent();
            return;
        }
        if (choice == 101 || choice == 102) {
            if (!commitSelectedComponentEdits()) return;
            std::uint32_t bypassId = 0;
            bool fitted = false;
            if (!editNetwork([id, choice, &bypassId, &fitted](ExhaustNetworkConfig& network) {
                    if (choice == 101) return ExhaustNetworkEditor::addValvedBypass(network, id, &bypassId);
                    return toggleInletValve(network, id, fitted);
                }))
                return;
            if (choice == 101) {
                selectedComponentId_ = bypassId;
                setStatus("#" + juce::String(id) + " has a bypass, pipe #" + juce::String(bypassId)
                              + ", behind an active valve: shut, the gas goes through #" + juce::String(id)
                              + "; open, most of it takes the bypass. Drive switches it.", false);
            } else {
                selectedComponentId_ = id;
                setStatus(fitted ? "#" + juce::String(id) + " has an active valve at its inlet."
                                 : "#" + juce::String(id) + " has no valve any more.", false);
            }
            rebuildAll();
            return;
        }
        if (choice == 103) {
            showValveSettings();
            return;
        }
        if (choice < 1 || choice > static_cast<int>(ExhaustComponentType::crossover) + 1) return;
        if (!commitSelectedComponentEdits()) return;
        const auto type = static_cast<ExhaustComponentType>(choice - 1);
        if (!editNetwork([id, type](ExhaustNetworkConfig& network) {
                return ExhaustNetworkEditor::replaceComponent(network, id, type);
            }))
            return;
        selectedComponentId_ = id;
        setStatus("#" + juce::String(id) + " is now a " + juce::String(componentTypeName(type)).toLowerCase()
                      + ": its connections, bore and length were kept.", false);
        rebuildAll();
    }

    /** Fits or removes the valve at a pipe's inlet; it needs one part, not a
     *  cylinder, feeding the pipe (validateEngineConfig's rule). */
    [[nodiscard]] static std::string toggleInletValve(ExhaustNetworkConfig& network, std::uint32_t id,
                                                      bool& fitted) {
        const auto found = std::find_if(network.components.begin(), network.components.end(),
            [id](const ExhaustComponentConfig& component) { return component.id == id; });
        if (found == network.components.end()) return "That component is not in this path.";
        if (found->inletValve) {
            found->inletValve = false;
            fitted = false;
            return {};
        }
        if (found->type != ExhaustComponentType::pipe) return "An active valve sits at a pipe's inlet.";
        for (const auto& cylinder : network.cylinderConnections)
            if (cylinder.componentId == id)
                return "A cylinder feeds this pipe: a valve goes on a pipe fed by another part.";
        const auto inputs = std::count_if(network.connections.begin(), network.connections.end(),
            [id](const ExhaustComponentConnectionConfig& connection) { return connection.toComponentId == id; });
        if (inputs != 1) return "A valve's pipe is fed by exactly one other part.";
        found->inletValve = true;
        fitted = true;
        return {};
    }

    /** The actuator: when automatic opens the valve, and how long it takes. */
    void showValveSettings() {
        if (valveDialog_) return;
        const auto& valve = working_.exhaustValve;
        valveDialog_ = std::make_unique<juce::AlertWindow>("Active exhaust valve",
            "Automatic opens every valve of this exhaust at or above either the engine speed or the "
            "throttle, and shuts it 250 rpm and 5 % below both. The stroke takes the travel time "
            "(0 moves at once). Drive's switch can also hold it shut or open.",
            juce::MessageBoxIconType::NoIcon);
        valveDialog_->addTextEditor("rpm", juce::String(valve.openAboveRpm, 0), "Opens above (rpm)");
        valveDialog_->addTextEditor("throttle", juce::String(valve.openAboveThrottle * 100.0, 0),
                                    "or above throttle (%)");
        valveDialog_->addTextEditor("travel", juce::String(valve.travelSeconds, 2), "Travel time (s)");
        valveDialog_->addButton("OK", 1, juce::KeyPress(juce::KeyPress::returnKey));
        valveDialog_->addButton("CANCEL", 0, juce::KeyPress(juce::KeyPress::escapeKey));
        ui::placeInWindow(*valveDialog_, *this);
        auto safe = juce::Component::SafePointer<DesignerContent>(this);
        valveDialog_->enterModalState(true, juce::ModalCallbackFunction::create([safe](int result) {
            if (!safe) return;
            double values[3] {};
            bool parsed = true;
            const char* names[3] { "rpm", "throttle", "travel" };
            for (int index = 0; index < 3; ++index) {
                const auto* editor = safe->valveDialog_ ? safe->valveDialog_->getTextEditor(names[index]) : nullptr;
                parsed = parsed && editor != nullptr && parseFinite(editor->getText(), values[index]);
            }
            safe->valveDialog_.reset();
            if (result != 1) return;
            if (!parsed || values[0] < 0.0 || values[0] > 100'000.0 || values[1] < 0.0 || values[1] > 100.0
                || values[2] < 0.0 || values[2] > 60.0) {
                safe->setStatus("Valve settings: 0-100,000 rpm, 0-100 % throttle, 0-60 s of travel.", true);
                return;
            }
            safe->rememberForUndo();
            safe->working_.exhaustValve.openAboveRpm = values[0];
            safe->working_.exhaustValve.openAboveThrottle = values[1] / 100.0;
            safe->working_.exhaustValve.travelSeconds = values[2];
            safe->setStatus("Valve: opens above " + juce::String(values[0], 0) + " rpm or "
                                + juce::String(values[1], 0) + " % throttle, " + juce::String(values[2], 2)
                                + " s of travel. Apply to hear it.", false);
        }), false);
    }

    void showSourceMenu(const std::vector<std::uint32_t>& cylinders) {
        if (cylinders.empty()) return;
        juce::PopupMenu menu;
        const auto rotor = rotorOf(cylinders.front());
        menu.addSectionHeader(rotor >= 0 ? "Rotor " + juce::String(rotor + 1) : cylinderName(cylinders.front()));
        menu.addItem(1, "Disconnect");
        menu.showMenuAsync(menuAtMouse(),
            [safe = juce::Component::SafePointer<DesignerContent>(this), cylinders](int choice) {
                if (safe == nullptr || choice != 1) return;
                if (!safe->commitSelectedComponentEdits()) return;
                if (!safe->editNetwork([&cylinders](ExhaustNetworkConfig& network) {
                        for (const auto cylinderId : cylinders) ExhaustNetworkEditor::disconnectCylinder(network, cylinderId);
                        return std::string {};
                    }))
                    return;
                safe->selectedMappingRow_ = -1;
                safe->setStatus("Disconnected: drag the cylinder onto a part, or pick it in From.", false);
                safe->rebuildAll();
            });
    }

    void rebuildComponentSelectors() {
        const auto preserveFrom = selectedId(fromSelector_);
        const auto preserveFromCylinder = selectedFromCylinder();
        const auto preserveTo = selectedId(toSelector_);
        const auto preserveMapping = selectedId(mappingComponentSelector_);
        for (auto* selector : std::array<juce::ComboBox*, 3> {
                 &fromSelector_, &toSelector_, &mappingComponentSelector_ })
            selector->clear(juce::dontSendNotification);
        const auto* network = selectedNetwork();
        if (network == nullptr) return;
        for (std::size_t index = 0; index < network->components.size(); ++index) {
            const auto& component = network->components[index];
            const auto label = "#" + juce::String(component.id) + " " + componentTypeName(component.type);
            const auto itemId = static_cast<int>(index) + 1;
            fromSelector_.addItem(label, itemId);
            toSelector_.addItem(label, itemId);
            mappingComponentSelector_.addItem(label, itemId);
        }
        // A cylinder can feed any part: it is a source like a component
        // (owner report 2026-11-11), listed after them so a component's
        // index is still its row.
        if (const auto* path = selectedPath(); path != nullptr && !path->cylinderIds.empty()) {
            fromSelector_.addSectionHeading("Cylinders");
            for (std::size_t index = 0; index < path->cylinderIds.size(); ++index)
                fromSelector_.addItem(cylinderName(path->cylinderIds[index]), cylinderItemBase + static_cast<int>(index));
            if (preserveFromCylinder) {
                const auto found = std::find(path->cylinderIds.begin(), path->cylinderIds.end(), *preserveFromCylinder);
                if (found != path->cylinderIds.end())
                    fromSelector_.setSelectedId(
                        cylinderItemBase + static_cast<int>(std::distance(path->cylinderIds.begin(), found)),
                        juce::dontSendNotification);
            }
        }
        restoreId(fromSelector_, preserveFrom);
        restoreId(toSelector_, preserveTo);
        restoreId(mappingComponentSelector_, preserveMapping);
    }

    /** The cylinder picked in From, if a cylinder is. */
    [[nodiscard]] std::optional<std::uint32_t> selectedFromCylinder() const {
        const auto* path = selectedPath();
        const auto itemId = fromSelector_.getSelectedId();
        if (path == nullptr || itemId < cylinderItemBase) return std::nullopt;
        const auto index = static_cast<std::size_t>(itemId - cylinderItemBase);
        return index < path->cylinderIds.size() ? std::optional(path->cylinderIds[index]) : std::nullopt;
    }

    void rebuildCylinderSelector() {
        const auto previousIndex = cylinderSelector_.getSelectedItemIndex();
        cylinderSelector_.clear(juce::dontSendNotification);
        const auto* path = selectedPath();
        if (path == nullptr) return;
        for (std::size_t index = 0; index < path->cylinderIds.size(); ++index)
            cylinderSelector_.addItem(cylinderName(path->cylinderIds[index]),
                                      static_cast<int>(index) + 1);
        if (!path->cylinderIds.empty())
            cylinderSelector_.setSelectedItemIndex(std::clamp(previousIndex, 0,
                static_cast<int>(path->cylinderIds.size()) - 1), juce::dontSendNotification);
    }

    [[nodiscard]] std::optional<std::uint32_t> selectedId(const juce::ComboBox& selector) const {
        const auto index = selector.getSelectedItemIndex();
        return componentAt(index) != nullptr ? std::optional(componentAt(index)->id) : std::nullopt;
    }

    void restoreId(juce::ComboBox& selector, std::optional<std::uint32_t> id) {
        if (!id) return;
        const auto* network = selectedNetwork();
        if (network == nullptr) return;
        const auto found = std::find_if(network->components.begin(), network->components.end(),
            [id](const auto& component) { return component.id == *id; });
        if (found != network->components.end())
            selector.setSelectedItemIndex(static_cast<int>(std::distance(network->components.begin(), found)),
                                          juce::dontSendNotification);
    }

    void selectFirstComponent() {
        const auto* network = selectedNetwork();
        if (network != nullptr && !network->components.empty())
            selectedComponentId_ = network->components.front().id;
    }

    void selectComponentId(std::uint32_t id) {
        if (!trySelectComponent(id)) {
            graphCanvas_.repaint();
            return;
        }
        const auto* network = selectedNetwork();
        if (network == nullptr) return;
        const auto found = std::find_if(network->components.begin(), network->components.end(),
            [id](const auto& component) { return component.id == id; });
        if (found != network->components.end()) {
            suppressComponentSelection_ = true;
            componentList_.selectRow(static_cast<int>(std::distance(network->components.begin(), found)));
            suppressComponentSelection_ = false;
        }
    }

    void selectComponentRow(int row) {
        if (suppressComponentSelection_) return;
        const auto* component = componentAt(row);
        if (component == nullptr) return;
        const auto targetId = component->id;
        if (!trySelectComponent(targetId)) {
            const auto* network = selectedNetwork();
            if (network == nullptr) return;
            const auto current = std::find_if(network->components.begin(), network->components.end(),
                [this](const ExhaustComponentConfig& item) {
                    return item.id == selectedComponentId_;
                });
            if (current != network->components.end()) {
                suppressComponentSelection_ = true;
                componentList_.selectRow(
                    static_cast<int>(std::distance(network->components.begin(), current)));
                suppressComponentSelection_ = false;
            }
        }
    }

    [[nodiscard]] bool trySelectComponent(std::uint32_t targetId) {
        if (targetId == selectedComponentId_) {
            graphCanvas_.repaint();
            return true;
        }
        if (!commitSelectedComponentEdits()) return false;
        const auto* network = selectedNetwork();
        if (network == nullptr) return false;
        const auto target = std::find_if(network->components.begin(), network->components.end(),
            [targetId](const ExhaustComponentConfig& item) { return item.id == targetId; });
        if (target == network->components.end()) return false;
        selectedComponentId_ = targetId;
        loadSelectedComponent();
        graphCanvas_.repaint();
        return true;
    }

    [[nodiscard]] int componentCount() const {
        const auto* network = selectedNetwork();
        return network != nullptr ? static_cast<int>(network->components.size()) : 0;
    }

    [[nodiscard]] juce::String componentText(int row) const {
        const auto* component = componentAt(row);
        if (component == nullptr) return {};
        auto diameter = juce::String(component->diameterMm, 1);
        if (component->outletDiameterMm > 0.0)
            diameter += " -> " + juce::String(component->outletDiameterMm, 1);
        return "#" + juce::String(component->id) + "  " + componentTypeName(component->type)
            + (component->inletValve ? " + valve" : "") + "  |  D " + diameter + " mm";
    }

    [[nodiscard]] int connectionCount() const {
        const auto* network = selectedNetwork();
        return network != nullptr ? static_cast<int>(network->connections.size()) : 0;
    }

    [[nodiscard]] juce::String connectionText(int row) const {
        const auto* network = selectedNetwork();
        if (network == nullptr || row < 0 || row >= static_cast<int>(network->connections.size())) return {};
        const auto& connection = network->connections[static_cast<std::size_t>(row)];
        auto from = "#" + juce::String(connection.fromComponentId);
        auto to = "#" + juce::String(connection.toComponentId);
        if (connection.fromPort != unspecifiedExhaustComponentPort)
            from += "[port " + juce::String(
                static_cast<int>(connection.fromPort)) + "]";
        if (connection.toPort != unspecifiedExhaustComponentPort)
            to += "[port " + juce::String(
                static_cast<int>(connection.toPort)) + "]";
        return from + "  ->  " + to;
    }

    [[nodiscard]] int mappingCount() const {
        const auto* network = selectedNetwork();
        return network != nullptr ? static_cast<int>(network->cylinderConnections.size()) : 0;
    }

    [[nodiscard]] juce::String mappingText(int row) const {
        const auto* network = selectedNetwork();
        if (network == nullptr || row < 0
            || row >= static_cast<int>(network->cylinderConnections.size())) return {};
        const auto& mapping = network->cylinderConnections[static_cast<std::size_t>(row)];
        return cylinderName(mapping.cylinderId) + "  ->  #" + juce::String(mapping.componentId);
    }

    /** A face's rotor (from 0): a rotary runs each rotor as three faces, in
     *  order; -1 for a piston engine's cylinder. */
    [[nodiscard]] int rotorOf(std::uint32_t cylinderId) const {
        if (working_.cycle != EngineCycle::rotary) return -1;
        const auto found = std::find_if(working_.cylinders.begin(), working_.cylinders.end(),
            [cylinderId](const CylinderConfig& cylinder) { return cylinder.id == cylinderId; });
        return found == working_.cylinders.end() ? -1
            : static_cast<int>(std::distance(working_.cylinders.begin(), found) / 3);
    }

    [[nodiscard]] juce::String cylinderName(std::uint32_t cylinderId) const {
        const auto rotor = rotorOf(cylinderId);
        if (rotor < 0) return "Cylinder " + juce::String(cylinderId);
        const auto index = std::find_if(working_.cylinders.begin(), working_.cylinders.end(),
            [cylinderId](const CylinderConfig& cylinder) { return cylinder.id == cylinderId; })
            - working_.cylinders.begin();
        return "Rotor " + juce::String(rotor + 1) + ", face " + juce::String(static_cast<int>(index % 3) + 1);
    }

    void selectMappingRow(int row) {
        selectedMappingRow_ = row;
        const auto* network = selectedNetwork();
        const auto* path = selectedPath();
        if (network == nullptr || path == nullptr || row < 0
            || row >= static_cast<int>(network->cylinderConnections.size())) return;
        const auto& mapping = network->cylinderConnections[static_cast<std::size_t>(row)];
        const auto cylinder = std::find(path->cylinderIds.begin(), path->cylinderIds.end(), mapping.cylinderId);
        if (cylinder != path->cylinderIds.end())
            cylinderSelector_.setSelectedItemIndex(static_cast<int>(std::distance(path->cylinderIds.begin(), cylinder)),
                                                    juce::dontSendNotification);
        restoreId(mappingComponentSelector_, mapping.componentId);
    }

    void loadSelectedComponent() {
        const auto* component = selectedComponent();
        const auto enabled = component != nullptr;
        componentTypeSelector_.setEnabled(enabled);
        updateComponentButton_.setEnabled(enabled);
        deleteComponentButton_.setEnabled(enabled);
        packingDemoButton_.setEnabled(enabled);
        packingBypassButton_.setEnabled(enabled);
        for (auto& editor : propertyEditors_) editor.setEnabled(enabled);
        if (!enabled) {
            componentTypeSelector_.setSelectedItemIndex(-1, juce::dontSendNotification);
            for (auto& editor : propertyEditors_) editor.clear();
            return;
        }
        componentTypeSelector_.setSelectedItemIndex(static_cast<int>(component->type),
                                                     juce::dontSendNotification);
        const auto values = shownValues(*component);
        propertyEditors_[0].setText(juce::String(component->id), false);
        for (std::size_t index = 0; index < values.size(); ++index)
            propertyEditors_[index + 1].setText(shownText(index, values[index]), false);
        updatePackingEditorAvailability(false);
    }

    /** What the property fields 1..11 show of a component. */
    [[nodiscard]] static std::array<double, 11> shownValues(const ExhaustComponentConfig& component) {
        std::array<double, 11> values {
            component.lengthMm, component.diameterMm, component.outletDiameterMm,
            component.volumeLitres,
            component.restriction, component.resonanceHz, component.acousticGain,
            component.dischargeCoefficient,
            component.packingFlowResistivityPaSPerM2,
            component.packingThicknessMm,
            component.perforatedOpenAreaRatio
        };
        if (component.type == ExhaustComponentType::crossover)
            values[5] = component.crossoverCoupling;
        if (component.type == ExhaustComponentType::catalyst) {
            values[8] = component.catalystCellDensityCpsi;
            values[9] = component.catalystOpenAreaRatio;
            values[10] = component.catalystSubstrateVolumetricHeatCapacityJPerM3K;
        }
        return values;
    }

    [[nodiscard]] static juce::String shownText(std::size_t index, double value) {
        return juce::String(value, index == 0 ? 1 : 3);
    }

    void layoutPropertyColumn(juce::Rectangle<int> bounds,
                              std::initializer_list<int> indices) {
        for (const auto index : indices) {
            auto row = bounds.removeFromTop(43);
            propertyLabels_[static_cast<std::size_t>(index)].setBounds(row.removeFromTop(17));
            propertyEditors_[static_cast<std::size_t>(index)].setBounds(row.removeFromTop(24));
            bounds.removeFromTop(1);
        }
    }

    void updatePackingEditorAvailability(bool clearIfUnavailable) {
        const auto selectedType = componentTypeSelector_.getSelectedItemIndex();
        const auto available = selectedType
            == static_cast<int>(ExhaustComponentType::muffler);
        const auto catalyst = selectedType
            == static_cast<int>(ExhaustComponentType::catalyst);
        const auto crossover = selectedType
            == static_cast<int>(ExhaustComponentType::crossover);
        propertyLabels_[2].setText(available
                ? "Packed core / dry chamber diameter (mm)"
                : "Inlet diameter (mm)",
            juce::dontSendNotification);
        propertyLabels_[4].setText(available
                ? "Gross shell volume (L)" : "Volume (L)",
            juce::dontSendNotification);
        propertyLabels_[6].setText(crossover
                ? "Cross-coupling k (0..1)"
                : "Branch tuning (Hz, 0 = length)",
            juce::dontSendNotification);
        propertyLabels_[9].setText(catalyst
                ? "Cell density (cpsi)"
                : "Packing resistivity (Pa.s/m2)",
            juce::dontSendNotification);
        propertyLabels_[10].setText(catalyst
                ? "Monolith open area (0..1)"
                : "Packing thickness (mm)",
            juce::dontSendNotification);
        propertyLabels_[11].setText(catalyst
                ? "Substrate vol. heat capacity (J/m3/K)"
                : "Perforate open area (0..1)",
            juce::dontSendNotification);
        if (clearIfUnavailable) {
            propertyEditors_[9].setText(catalyst ? "400" : "0", false);
            propertyEditors_[10].setText(catalyst ? "0.8" : "0", false);
            propertyEditors_[11].setText(catalyst ? "2000000" : "0", false);
        }
        for (std::size_t index = 9; index < propertyEditors_.size(); ++index) {
            const auto enabled = available || catalyst;
            propertyEditors_[index].setEnabled(enabled);
        }
        const auto tunableBranch = selectedType
            == static_cast<int>(ExhaustComponentType::resonator);
        propertyEditors_[6].setEnabled(tunableBranch || crossover);
        if (crossover && clearIfUnavailable)
            propertyEditors_[6].setText("0.3", false);
        else if (!tunableBranch && clearIfUnavailable)
            propertyEditors_[6].setText("0", false);
        packingDemoButton_.setEnabled(available);
        packingBypassButton_.setEnabled(available);
    }

    void setPackingPreset(bool enabled) {
        if (componentTypeSelector_.getSelectedItemIndex()
                != static_cast<int>(ExhaustComponentType::muffler)) {
            setStatus("Porous packing only applies to a muffler.", true);
            return;
        }
        propertyEditors_[9].setText(enabled ? "24000" : "0", false);
        propertyEditors_[10].setText(enabled ? "35" : "0", false);
        propertyEditors_[11].setText(enabled ? "0.28" : "0", false);
        updateSelectedComponent();
    }

    void generateLegacyNetwork() {
        auto* path = selectedPath();
        if (path == nullptr || path->cylinderIds.empty()) {
            setStatus("Cannot generate a network: this path has no cylinder.", true);
            return;
        }
        rememberForUndo();
        path->network = makeEditableExhaustNetwork(*path, working_.cylinders);
        selectedComponentId_ = path->network->components.front().id;
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        setStatus("Simple network generated from the legacy geometry. Check it, then apply.", false);
        rebuildAll();
    }

    void addComponent() {
        auto* path = selectedPath();
        if (path == nullptr) return;
        if (path->network && path->network->components.size() >= maximumComponents) {
            setStatus("Limit reached: 256 components per path.", true);
            return;
        }
        const auto typeIndex = addTypeSelector_.getSelectedItemIndex();
        if (typeIndex < 0) {
            setStatus("Select a component type.", true);
            return;
        }
        const auto id = path->network ? ExhaustNetworkEditor::nextComponentId(*path->network) : 1U;
        if (id == 0) {
            setStatus("No component identifier available.", true);
            return;
        }
        rememberForUndo();
        if (!path->network) path->network.emplace();
        auto& network = *path->network;
        const auto type = static_cast<ExhaustComponentType>(typeIndex);
        network.components.push_back(ExhaustNetworkEditor::defaultComponent(type, id));
        selectedComponentId_ = id;
        setStatus(type == ExhaustComponentType::resonator
                ? "Resonator added: one inlet and no outlet make a closed branch; adding an outlet makes it inline."
                : "Component added. Connect it before validating the network.",
            false);
        rebuildAll();
        componentList_.selectRow(static_cast<int>(network.components.size()) - 1);
    }

    void deleteComponent() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedComponent() == nullptr) return;
        const auto id = selectedComponentId_;
        rememberForUndo();
        std::erase_if(network->components, [id](const auto& component) { return component.id == id; });
        std::erase_if(network->connections, [id](const auto& connection) {
            return connection.fromComponentId == id || connection.toComponentId == id;
        });
        std::erase_if(network->cylinderConnections, [id](const auto& mapping) {
            return mapping.componentId == id;
        });
        selectedComponentId_ = 0;
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        selectFirstComponent();
        setStatus("Component and its references deleted.", false);
        rebuildAll();
    }

    void updateSelectedComponent() {
        auto* component = selectedComponent();
        auto* network = selectedNetwork();
        if (component == nullptr || network == nullptr) return;

        std::uint32_t newId = 0;
        if (!parseId(propertyEditors_[0].getText(), newId)) {
            setStatus("The ID must be a unique integer between 1 and 4294967295.", true);
            return;
        }
        if (newId != component->id && std::any_of(network->components.begin(), network->components.end(),
                [newId](const auto& candidate) { return candidate.id == newId; })) {
            setStatus("This identifier is already used in the path.", true);
            return;
        }
        const auto current = shownValues(*component);
        std::array<double, 11> values {};
        for (std::size_t index = 0; index < values.size(); ++index) {
            // A field left as shown keeps its exact value: the display rounds
            // it (763.27 mm read back as 763.3 on every selection change).
            const auto text = propertyEditors_[index + 1].getText();
            if (text == shownText(index, current[index])) {
                values[index] = current[index];
                continue;
            }
            if (!parseFinite(text, values[index])) {
                setStatus("Every property must be a finite number.", true);
                return;
            }
        }
        const auto typeIndex = componentTypeSelector_.getSelectedItemIndex();
        if (typeIndex < static_cast<int>(ExhaustComponentType::pipe)
            || typeIndex > static_cast<int>(ExhaustComponentType::crossover)) {
            setStatus("Invalid component type.", true);
            return;
        }
        const auto type = static_cast<ExhaustComponentType>(typeIndex);
        const auto requiresLength = type == ExhaustComponentType::pipe
            || type == ExhaustComponentType::resonator
            || type == ExhaustComponentType::muffler
            || type == ExhaustComponentType::catalyst;
        if (values[0] < (requiresLength ? 1.0 : 0.0) || values[0] > 10'000.0
            || values[1] < 5.0 || values[1] > 500.0
            || !(values[2] == 0.0 || (values[2] >= 5.0 && values[2] <= 500.0))
            || values[3] < 0.0 || values[3] > 1'000.0
            || values[4] < 0.0 || values[4] > 20.0
            || values[5] < 0.0
            || values[5] > (type == ExhaustComponentType::crossover
                    ? 1.0 : 20'000.0)
            || values[6] < 0.0 || values[6] > 8.0
            || values[7] < 0.02 || values[7] > 1.5
            || values[8] < 0.0 || values[8] > 200'000.0
            || values[9] < 0.0 || values[9] > 300.0
            || values[10] < 0.0 || values[10] > 10'000'000.0) {
            setStatus("Values out of range (L 0..10000, inlet D 5..500, outlet D 0 or 5..500, V 0..1000, restriction 0..20, "
                      "resonance 0..20000, gain 0..8, Cd 0.02..1.5).", true);
            return;
        }
        const auto hasNoPacking = values[8] == 0.0
            && values[9] == 0.0 && values[10] == 0.0;
        const auto hasCompletePacking = type == ExhaustComponentType::muffler
            && values[8] > 0.0 && values[9] > 0.0
            && values[10] > 0.0 && values[10] <= 1.0;
        const auto monolith = catalystMonolithGeometry(values[8], values[9]);
        const auto hasCompleteMonolith = type == ExhaustComponentType::catalyst
            && values[8] >= 25.0 && values[8] <= 5'000.0
            && values[9] >= 0.05 && values[9] <= 0.99
            && values[10] >= 100'000.0 && values[10] <= 10'000'000.0
            && monolith.active
            && monolith.cellPitchM <= values[1] * 0.001;
        if (!hasNoPacking && !hasCompletePacking && !hasCompleteMonolith) {
            setStatus("Invalid material: complete packing on a muffler, or a catalyst with 25..5000 cpsi, open area 0.05..0.99 and heat capacity 0.1..10 MJ/m3/K.", true);
            return;
        }
        if (type == ExhaustComponentType::crossover
            && (values[0] != 0.0 || values[2] != 0.0
                || values[3] != 0.0)) {
            setStatus("A compact X-pipe needs zero length, outlet diameter and volume; draw its four legs with neighbouring pipes.", true);
            return;
        }
        if (hasCompletePacking) {
            constexpr double pi = 3.14159265358979323846;
            const auto inletRadiusM = values[1] * 0.0005;
            const auto outletRadiusM = (values[2] > 0.0 ? values[2] : values[1])
                * 0.0005;
            const auto inletAreaM2 = pi * inletRadiusM * inletRadiusM;
            const auto outletAreaM2 = pi * outletRadiusM * outletRadiusM;
            const auto coreMeanAreaM2 = (inletAreaM2
                + std::sqrt(inletAreaM2 * outletAreaM2)
                + outletAreaM2) / 3.0;
            const auto coreVolumeLitres = coreMeanAreaM2 * values[0];
            if (!(values[3] > coreVolumeLitres * (1.0 + 1.0e-9))) {
                setStatus("Invalid packed muffler: the gross shell volume must exceed the volume of the perforated core.", true);
                return;
            }
        }

        // Committed on every selection change: only a real edit is an undo step.
        auto before = snapshot();
        const auto previous = *component;
        const auto oldId = component->id;
        const auto oldType = component->type;
        component->id = newId;
        component->type = type;
        component->lengthMm = values[0];
        component->diameterMm = values[1];
        component->outletDiameterMm = values[2];
        component->volumeLitres = values[3];
        component->restriction = values[4];
        component->resonanceHz = type == ExhaustComponentType::crossover
            ? 0.0 : values[5];
        component->crossoverCoupling = type == ExhaustComponentType::crossover
            ? values[5] : 0.0;
        component->acousticGain = values[6];
        component->dischargeCoefficient = values[7];
        component->packingFlowResistivityPaSPerM2 = hasCompletePacking ? values[8] : 0.0;
        component->packingThicknessMm = hasCompletePacking ? values[9] : 0.0;
        component->perforatedOpenAreaRatio = hasCompletePacking ? values[10] : 0.0;
        component->catalystCellDensityCpsi = hasCompleteMonolith ? values[8] : 0.0;
        component->catalystOpenAreaRatio = hasCompleteMonolith ? values[9] : 0.0;
        component->catalystSubstrateVolumetricHeatCapacityJPerM3K =
            hasCompleteMonolith ? values[10] : 0.0;
        if (newId != oldId) {
            for (auto& connection : network->connections) {
                if (connection.fromComponentId == oldId) connection.fromComponentId = newId;
                if (connection.toComponentId == oldId) connection.toComponentId = newId;
            }
            for (auto& mapping : network->cylinderConnections)
                if (mapping.componentId == oldId) mapping.componentId = newId;
        }
        if (oldType != type) {
            auto nextInputPort = std::uint8_t { 0 };
            auto nextOutputPort = std::uint8_t { 0 };
            // A merge and a splitter keep their side branch (an H's balance pipe).
            const auto junction = type == ExhaustComponentType::merge || type == ExhaustComponentType::splitter;
            const auto port = [&](std::uint8_t authored, std::uint8_t& next) {
                if (type == ExhaustComponentType::crossover && next <= 1U) return next++;
                return junction && authored == exhaustSideBranchPort ? authored : unspecifiedExhaustComponentPort;
            };
            for (auto& connection : network->connections) {
                if (connection.toComponentId == newId) connection.toPort = port(connection.toPort, nextInputPort);
                if (connection.fromComponentId == newId) connection.fromPort = port(connection.fromPort, nextOutputPort);
            }
        }
        const auto edited = !sameEditedFields(previous, *component);
        selectedComponentId_ = newId;
        if (edited) pushUndo(std::move(before));
        setStatus(edited ? "Component updated in the working copy." : "Component unchanged.", false);
        rebuildAll();
    }

    void addConnection() {
        const auto from = selectedId(fromSelector_);
        const auto cylinder = selectedFromCylinder();
        const auto to = selectedId(toSelector_);
        if (selectedNetwork() == nullptr || (!from && !cylinder) || !to) {
            setStatus("Select a source (a component or a cylinder) and a destination component.", true);
            return;
        }
        if (cylinder) connectCylinders({ *cylinder }, *to);
        else connectComponents(*from, *to);
    }

    void deleteConnection() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedConnectionRow_ < 0
            || selectedConnectionRow_ >= static_cast<int>(network->connections.size())) {
            setStatus("Select a connection to delete.", true);
            return;
        }
        rememberForUndo();
        network->connections.erase(network->connections.begin() + selectedConnectionRow_);
        selectedConnectionRow_ = -1;
        setStatus("Connection deleted.", false);
        rebuildAll();
    }

    void assignCylinder() {
        auto* network = selectedNetwork();
        const auto* path = selectedPath();
        const auto componentId = selectedId(mappingComponentSelector_);
        const auto cylinderIndex = cylinderSelector_.getSelectedItemIndex();
        if (network == nullptr || path == nullptr || !componentId || cylinderIndex < 0
            || cylinderIndex >= static_cast<int>(path->cylinderIds.size())) {
            setStatus("Select a cylinder and its first component.", true);
            return;
        }
        const auto cylinderId = path->cylinderIds[static_cast<std::size_t>(cylinderIndex)];
        if (!editNetwork([cylinderId, componentId](ExhaustNetworkConfig& edited) {
                return ExhaustNetworkEditor::connectCylinder(edited, cylinderId, *componentId);
            }))
            return;
        const auto mapped = std::find_if(network->cylinderConnections.begin(),
            network->cylinderConnections.end(), [cylinderId](const auto& mapping) {
                return mapping.cylinderId == cylinderId;
            });
        selectedMappingRow_ = static_cast<int>(std::distance(network->cylinderConnections.begin(), mapped));
        setStatus("Cylinder assignment updated.", false);
        rebuildAll();
        mappingList_.selectRow(selectedMappingRow_);
    }

    void deleteMapping() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedMappingRow_ < 0
            || selectedMappingRow_ >= static_cast<int>(network->cylinderConnections.size())) {
            setStatus("Select an assignment to delete.", true);
            return;
        }
        rememberForUndo();
        network->cylinderConnections.erase(network->cylinderConnections.begin() + selectedMappingRow_);
        selectedMappingRow_ = -1;
        setStatus("Assignment deleted; the cylinder must be reassigned before validation.", false);
        rebuildAll();
    }

    /** What the applied network costs the solver, against the one it replaced.
     *
     * Advisory, never a refusal: an unusual exhaust is a legitimate thing to
     * want, and the editor has no business deciding otherwise. What it does
     * have a business doing is not letting the cost be invisible -- the failure
     * this exists for looks nothing like a solver problem from the driver's
     * seat. It reads as the engine having gained inertia, gone dull and fallen
     * silent, because the physics thread is late and the cylinder-pressure
     * telemetry that excites the whole exhaust chain is produced slower than
     * the audio consumes it.
     */
    [[nodiscard]] juce::String solverCostAdvisory() const {
        const auto appliedM = minimumExhaustCflLengthM(working_);
        const auto baselineM = minimumExhaustCflLengthM(baseline_);
        if (!(appliedM > 0.0)) return {};
        auto text = "  Limiting CFL length " + juce::String(appliedM * 1'000.0, 1)
            + " mm";
        if (!(baselineM > 0.0)) return text + ".";
        // Rate is proportional to 1/dx for the same gas, so this ratio is an
        // exact ratio of solver cost with nothing assumed about the gas state.
        const auto costRatio = baselineM / appliedM;
        text += " versus " + juce::String(baselineM * 1'000.0, 1)
            + " mm before";
        if (costRatio >= 1.5) {
            text += ": the exhaust solver will sub-step "
                + juce::String(costRatio, 1)
                + "x faster. Check the realtime factor;"
                  " one short element sets the time step of the WHOLE network.";
        } else if (costRatio <= 0.67) {
            text += ": solver cost divided by "
                + juce::String(1.0 / costRatio, 1) + ".";
        } else {
            text += " (solver cost unchanged).";
        }
        return text;
    }

    void apply() {
        if (selectedComponent() != nullptr) {
            updateSelectedComponent();
            if (statusIsError_) return;
        }
        if (const auto error = validateEngineConfig(working_)) {
            setStatus("Configuration rejected: " + utf8(*error), true);
            ui::showMessage(this, juce::MessageBoxIconType::WarningIcon,
                "Invalid exhaust network", utf8(*error));
            return;
        }
        if (!applyCallback_) {
            setStatus("Configuration valid, but no apply callback is installed.", true);
            return;
        }
        // Read the cost before the apply: setConfig makes the applied engine
        // the baseline, and the advisory always said "unchanged" (audit APP-02).
        const auto advisory = solverCostAdvisory();
        // The engine applied comes back through setConfig: keep the history.
        const juce::ScopedValueSetter<bool> applying(applying_, true);
        try {
            if (!applyCallback_(working_)) {
                setStatus("The valid configuration was rejected by the running engine.", true);
                return;
            }
            setStatus("Configuration validated and applied to the engine." + advisory, false);
        } catch (const std::exception& exception) {
            setStatus("Apply failed: " + utf8(exception.what()), true);
        } catch (...) {
            setStatus("Apply failed with an unknown error.", true);
        }
    }

    void setStatus(const juce::String& message, bool error) {
        statusIsError_ = error;
        statusLabel_.setText((error ? "ERROR  |  " : "INFO  |  ") + message,
                             juce::dontSendNotification);
        statusLabel_.setColour(juce::Label::textColourId,
            error ? juce::Colour(0xffff8177) : juce::Colour(0xff8fc7ae));
    }

    EngineConfig working_;
    EngineConfig baseline_;
    ApplyCallback applyCallback_;
    int selectedPathIndex_ {};
    std::uint32_t selectedComponentId_ {};
    int selectedConnectionRow_ { -1 };
    int selectedMappingRow_ { -1 };
    bool statusIsError_ { false };
    bool suppressComponentSelection_ { false };
    bool applying_ { false };
    std::vector<Snapshot> undo_;
    std::unique_ptr<juce::AlertWindow> valveDialog_;
    std::vector<Snapshot> redo_;

    CallbackListModel componentModel_;
    CallbackListModel connectionModel_;
    CallbackListModel mappingModel_;
    ExhaustGraphCanvas graphCanvas_;

    juce::GroupComponent graphGroup_;
    juce::GroupComponent componentGroup_;
    juce::GroupComponent propertyGroup_;
    juce::GroupComponent topologyGroup_;
    juce::ComboBox pathSelector_;
    juce::TextButton newPathButton_;
    juce::TextButton deletePathButton_;
    juce::TextButton legacyButton_;
    juce::TextButton hPipeButton_;
    juce::TextButton xPipeButton_;
    juce::TextButton applyButton_;
    juce::ComboBox pathCylinderSelector_;
    juce::ComboBox destinationPathSelector_;
    juce::TextButton moveCylinderButton_;
    juce::TextButton undoButton_;
    juce::TextButton redoButton_;
    juce::Label statusLabel_;

    juce::ListBox componentList_;
    juce::ComboBox addTypeSelector_;
    juce::TextButton addButton_;
    juce::TextButton deleteComponentButton_;

    juce::Label componentTypeLabel_;
    juce::ComboBox componentTypeSelector_;
    std::array<juce::Label, 12> propertyLabels_;
    std::array<juce::TextEditor, 12> propertyEditors_;
    juce::TextButton updateComponentButton_;
    juce::TextButton packingDemoButton_;
    juce::TextButton packingBypassButton_;

    juce::Label connectionTitle_;
    juce::ComboBox fromSelector_;
    juce::ComboBox toSelector_;
    juce::ListBox connectionList_;
    juce::TextButton addConnectionButton_;
    juce::TextButton deleteConnectionButton_;

    juce::Label mappingTitle_;
    juce::ComboBox cylinderSelector_;
    juce::ComboBox mappingComponentSelector_;
    juce::ListBox mappingList_;
    juce::TextButton assignCylinderButton_;
    juce::TextButton deleteMappingButton_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DesignerContent)
};

ExhaustDesignerWindow::ExhaustDesignerWindow(const EngineConfig& config,
                                               ApplyCallback applyCallback)
    : juce::DocumentWindow("EngineLab - Exhaust designer", juce::Colour(0xff0d1312),
                           juce::DocumentWindow::closeButton, true) {
    content_ = new DesignerContent(config, std::move(applyCallback));
    setContentOwned(content_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(1'100, 700, 2'200, 1'300);
    centreWithSize(1'420, 880);
}

ExhaustDesignerWindow::~ExhaustDesignerWindow() = default;

void ExhaustDesignerWindow::closeButtonPressed() { setVisible(false); }

bool ExhaustDesignerWindow::keyPressed(const juce::KeyPress& key) {
    // With nothing focused the window gets the keys: undo and redo still work.
    return content_ != nullptr && content_->keyPressed(key);
}

void ExhaustDesignerWindow::setConfig(const EngineConfig& config) {
    if (content_ != nullptr) content_->setConfig(config);
}

} // namespace enginelab
