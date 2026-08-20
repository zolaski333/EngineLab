#include <enginelab/app/ExhaustDesignerWindow.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

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
constexpr std::size_t maximumConnections = 1'024;

[[nodiscard]] juce::String utf8(std::string_view value) {
    return juce::String::fromUTF8(value.data(), static_cast<int>(value.size()));
}

/** Shortest meshed cell in an engine's compiled exhaust, metres; 0 if unknown.
 *
 * Validation checks ranges and topology and nothing else, so a network that is
 * perfectly legal can still be ruinously expensive: the explicit solver's time
 * step is set by the SHORTEST cell anywhere in the network, and every other
 * duct then substeps at that rate. A user who rebuilt a 2JZ exhaust with an
 * 80 x 10 mm silencer body reported that the engine had "gained inertia, lost
 * its liveliness and made almost no sound" -- three descriptions of one cause,
 * the network going from 11,520 Hz to 92,160 Hz of substepping and the physics
 * thread landing at 153 % of its frame budget. Nothing in the editor could
 * have told them.
 *
 * Returning the length rather than a frequency is deliberate: the substep RATE
 * is proportional to 1/dx for the same gas, so a ratio of two of these lengths
 * is an exact ratio of two solver costs, with no assumed sound speed anywhere.
 */
[[nodiscard]] double shortestExhaustCellM(const EngineConfig& config) {
    try {
        const auto graph = ExhaustGraph::makeForEngine(config);
        const auto layout = gasdynamics::ExhaustNetworkLayout::compile(graph);
        if (!layout.valid()) return 0.0;
        return layout.minimumCellLengthM();
    } catch (...) {
        return 0.0;
    }
}

[[nodiscard]] const char* componentTypeName(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe: return "Tube";
    case ExhaustComponentType::merge: return "Collecteur";
    case ExhaustComponentType::splitter: return "Separateur";
    case ExhaustComponentType::resonator: return "Resonateur";
    case ExhaustComponentType::muffler: return "Silencieux";
    case ExhaustComponentType::catalyst: return "Catalyseur";
    case ExhaustComponentType::outlet: return "Sortie";
    }
    return "Inconnu";
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

[[nodiscard]] ExhaustComponentConfig defaultComponent(ExhaustComponentType type,
                                                        std::uint32_t id) noexcept {
    ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    switch (type) {
    case ExhaustComponentType::pipe:
        result.lengthMm = 450.0;
        result.diameterMm = 42.0;
        break;
    case ExhaustComponentType::merge:
        result.lengthMm = 0.0;
        result.diameterMm = 58.0;
        result.volumeLitres = 1.5;
        break;
    case ExhaustComponentType::splitter:
        result.lengthMm = 0.0;
        result.diameterMm = 58.0;
        break;
    case ExhaustComponentType::resonator:
        // The most useful neutral starting point is a closed quarter-wave
        // branch. One incoming connection and no output gives it that meaning;
        // adding an output keeps the historical inline-duct behaviour.
        result.lengthMm = 535.0;
        result.diameterMm = 28.0;
        result.volumeLitres = 0.0;
        result.resonanceHz = 0.0;
        break;
    case ExhaustComponentType::muffler:
        result.lengthMm = 480.0;
        result.diameterMm = 62.0;
        result.volumeLitres = 8.0;
        result.restriction = 0.28;
        result.acousticGain = 0.72;
        break;
    case ExhaustComponentType::catalyst:
        result.lengthMm = 180.0;
        result.diameterMm = 56.0;
        result.volumeLitres = 1.2;
        result.restriction = 0.18;
        result.dischargeCoefficient = 0.68;
        break;
    case ExhaustComponentType::outlet:
        // A tailpipe is a pipe. Zero here used to mean "the system ends", but
        // the layout has to mesh it anyway, and a length it has to invent is
        // the length that then sets the CFL limit for the whole network. Give
        // it the real thing instead; the user can still shorten it.
        result.lengthMm = 300.0;
        result.diameterMm = 65.0;
        break;
    }
    return result;
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

class ExhaustGraphCanvas final : public juce::Component {
public:
    using PathProvider = std::function<const ExhaustPathConfig*()>;
    using SelectionProvider = std::function<std::uint32_t()>;
    using SelectionCallback = std::function<void(std::uint32_t)>;

    ExhaustGraphCanvas(PathProvider pathProvider, SelectionProvider selectionProvider,
                       SelectionCallback selectionCallback)
        : pathProvider_(std::move(pathProvider)), selectionProvider_(std::move(selectionProvider)),
          selectionCallback_(std::move(selectionCallback)) {}

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour(0xff101716));
        graphics.setColour(juce::Colour(0xff25312e));
        graphics.drawRect(getLocalBounds(), 1);
        nodeBounds_.clear();

        const auto* path = pathProvider_ ? pathProvider_() : nullptr;
        if (path == nullptr || !path->network || path->network->components.empty()) {
            graphics.setColour(juce::Colour(0xff83908c));
            graphics.setFont(juce::FontOptions(15.0F));
            graphics.drawFittedText("Aucun reseau personnalise. Generez le reseau legacy ou ajoutez des composants.",
                                    getLocalBounds().reduced(30), juce::Justification::centred, 3);
            return;
        }

        const auto& network = *path->network;
        std::unordered_map<std::uint32_t, int> levels;
        for (const auto& component : network.components) levels.emplace(component.id, 0);
        for (std::size_t pass = 0; pass < network.components.size(); ++pass) {
            bool changed = false;
            for (const auto& connection : network.connections) {
                const auto source = levels.find(connection.fromComponentId);
                const auto target = levels.find(connection.toComponentId);
                if (source == levels.end() || target == levels.end()) continue;
                const auto candidate = std::min(static_cast<int>(network.components.size()), source->second + 1);
                if (candidate > target->second) {
                    target->second = candidate;
                    changed = true;
                }
            }
            if (!changed) break;
        }

        int maximumLevel = 0;
        for (const auto& [id, level] : levels) {
            (void)id;
            maximumLevel = std::max(maximumLevel, level);
        }
        std::vector<std::vector<const ExhaustComponentConfig*>> columns(
            static_cast<std::size_t>(maximumLevel + 1));
        for (const auto& component : network.components)
            columns[static_cast<std::size_t>(levels[component.id])].push_back(&component);

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
            const juce::Point<float> start { source->second.getRight(), source->second.getCentreY() };
            const juce::Point<float> end { target->second.getX(), target->second.getCentreY() };
            juce::Path link;
            link.startNewSubPath(start);
            const auto controlOffset = std::max(14.0F, std::abs(end.x - start.x) * 0.38F);
            link.cubicTo(start.x + controlOffset, start.y, end.x - controlOffset, end.y, end.x, end.y);
            graphics.strokePath(link, juce::PathStrokeType(2.0F));
            graphics.fillEllipse(end.x - 3.5F, end.y - 3.5F, 7.0F, 7.0F);
        }

        const auto cylinderRows = std::max<std::size_t>(1U, path->cylinderIds.size());
        const auto cylinderSpacing = graph.getHeight() / static_cast<float>(cylinderRows);
        for (std::size_t index = 0; index < path->cylinderIds.size(); ++index) {
            const auto cylinderId = path->cylinderIds[index];
            const auto y = graph.getY() + (static_cast<float>(index) + 0.5F) * cylinderSpacing;
            const juce::Rectangle<float> pill(graph.getX(), y - 11.0F, cylinderWidth, 22.0F);
            graphics.setColour(juce::Colour(0xff263532));
            graphics.fillRoundedRectangle(pill, 5.0F);
            graphics.setColour(juce::Colour(0xffa9bbb5));
            graphics.setFont(juce::FontOptions(11.0F, juce::Font::bold));
            graphics.drawText("CYL " + juce::String(cylinderId), pill.toNearestInt(),
                              juce::Justification::centred);
            const auto mapping = std::find_if(network.cylinderConnections.begin(),
                network.cylinderConnections.end(), [cylinderId](const auto& candidate) {
                    return candidate.cylinderId == cylinderId;
                });
            if (mapping != network.cylinderConnections.end()) {
                if (const auto target = nodeBounds_.find(mapping->componentId); target != nodeBounds_.end()) {
                    graphics.setColour(juce::Colour(0xff60716c));
                    graphics.drawLine(pill.getRight(), pill.getCentreY(),
                                      target->second.getX(), target->second.getCentreY(), 1.5F);
                }
            }
        }

        const auto selectedId = selectionProvider_ ? selectionProvider_() : 0U;
        for (const auto& component : network.components) {
            auto bounds = nodeBounds_.at(component.id);
            const auto selected = selectedId == component.id;
            const auto colour = componentTypeColour(component.type);
            graphics.setColour(colour.withAlpha(selected ? 0.92F : 0.62F));
            graphics.fillRoundedRectangle(bounds, 7.0F);
            graphics.setColour(selected ? juce::Colour(0xffffd086) : colour.brighter(0.5F));
            graphics.drawRoundedRectangle(bounds, 7.0F, selected ? 2.5F : 1.0F);
            graphics.setColour(juce::Colours::white);
            graphics.setFont(juce::FontOptions(12.0F, juce::Font::bold));
            graphics.drawText("#" + juce::String(component.id) + "  " + componentTypeName(component.type),
                              bounds.removeFromTop(25.0F).toNearestInt(), juce::Justification::centred);
            graphics.setColour(juce::Colour(0xffe1e9e6));
            graphics.setFont(juce::FontOptions(10.0F));
            const auto detail = juce::String(component.diameterMm, 0) + " mm"
                + (component.lengthMm > 0.0 ? "  x  " + juce::String(component.lengthMm, 0) + " mm" : "");
            graphics.drawText(detail, bounds.toNearestInt(), juce::Justification::centred);
        }
    }

    void mouseDown(const juce::MouseEvent& event) override {
        for (const auto& [id, bounds] : nodeBounds_) {
            if (bounds.contains(event.position)) {
                if (selectionCallback_) selectionCallback_(id);
                return;
            }
        }
    }

private:
    PathProvider pathProvider_;
    SelectionProvider selectionProvider_;
    SelectionCallback selectionCallback_;
    std::unordered_map<std::uint32_t, juce::Rectangle<float>> nodeBounds_;
};

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
              [this](std::uint32_t id) { selectComponentId(id); }) {
        setOpaque(true);
        configureInterface();
        setConfig(config);
    }

    void setConfig(const EngineConfig& config) {
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
        setStatus("Copie de travail chargee. Aucune modification n'est encore appliquee.", false);
        rebuildAll();
    }

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour(0xff0d1312));
        const auto header = juce::Rectangle<int>(0, 0, getWidth(), 54);
        graphics.setGradientFill(juce::ColourGradient(juce::Colour(0xff1d2b27), 0.0F, 0.0F,
            juce::Colour(0xff111817), static_cast<float>(getWidth()), 0.0F, false));
        graphics.fillRect(header);
        graphics.setColour(juce::Colour(0xffdce7e3));
        graphics.setFont(juce::FontOptions(18.0F, juce::Font::bold));
        graphics.drawText("CONCEPTEUR D'ECHAPPEMENT", 14, 0, 280, 54,
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
        bounds.removeFromTop(5);

        statusLabel_.setBounds(bounds.removeFromBottom(29));
        bounds.removeFromBottom(8);

        auto graphArea = bounds.removeFromTop(std::clamp(bounds.getHeight() * 46 / 100, 260, 390));
        graphGroup_.setBounds(graphArea);
        graphCanvas_.setBounds(graphArea.reduced(10, 24));
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
        graphGroup_.setText("Vue du flux (cylindres vers sorties)");
        componentGroup_.setText("Composants");
        propertyGroup_.setText("Proprietes du composant");
        topologyGroup_.setText("Liaisons et affectations");
        for (auto* group : std::array<juce::GroupComponent*, 4> {
                 &graphGroup_, &componentGroup_, &propertyGroup_, &topologyGroup_ }) {
            group->setColour(juce::GroupComponent::outlineColourId, juce::Colour(0xff31413d));
            group->setColour(juce::GroupComponent::textColourId, juce::Colour(0xff9eb3ac));
            addAndMakeVisible(*group);
        }

        pathSelector_.setTextWhenNothingSelected("Chemin d'echappement");
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
        newPathButton_.setButtonText("+ CHEMIN");
        newPathButton_.setTooltip(
            "Cree un chemin et y deplace le cylindre selectionne, sans chemin vide.");
        newPathButton_.onClick = [this] { createPath(); };
        deletePathButton_.setButtonText("- CHEMIN");
        deletePathButton_.setTooltip(
            "Supprime le chemin courant et transfere tous ses cylindres vers la destination.");
        deletePathButton_.onClick = [this] { deletePath(); };
        legacyButton_.setButtonText("GENERER DEPUIS LEGACY");
        legacyButton_.setTooltip("Remplace le graphe du chemin selectionne par un reseau simple et valide.");
        legacyButton_.onClick = [this] { generateLegacyNetwork(); };
        applyButton_.setButtonText("VALIDER ET APPLIQUER");
        applyButton_.setColour(juce::TextButton::buttonColourId, juce::Colour(0xff477d68));
        applyButton_.onClick = [this] { apply(); };
        pathCylinderSelector_.setTextWhenNothingSelected("Cylindre a deplacer");
        destinationPathSelector_.setTextWhenNothingSelected("Chemin destination");
        moveCylinderButton_.setButtonText("DEPLACER");
        moveCylinderButton_.setTooltip(
            "Transfere le cylindre vers le chemin choisi et repare les racines du DAG custom.");
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
        addButton_.setButtonText("AJOUTER");
        addButton_.onClick = [this] { addComponent(); };
        deleteComponentButton_.setButtonText("SUPPRIMER");
        deleteComponentButton_.onClick = [this] { deleteComponent(); };

        componentTypeLabel_.setText("Type", juce::dontSendNotification);
        componentTypeLabel_.setColour(juce::Label::textColourId, juce::Colour(0xffaebbb7));
        populateTypeSelector(componentTypeSelector_);
        const std::array<const char*, 12> names {
            "ID", "Longueur (mm)", "Diametre entree (mm)",
            "Diametre sortie (mm, 0 = constant)", "Volume (L)",
            "Restriction debit (K)", "Accord branche (Hz, 0 = longueur)",
            "Gain fallback legacy", "Cd sortie",
            "Resistivite garnissage (Pa.s/m2)", "Epaisseur garnissage (mm)",
            "Taux ouvert perfore (0..1)"
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
            "Sur un resonateur terminal, ce volume est la cavite fermee au bout de la branche. "
            "Zero donne une terminaison rigide quart d'onde.");
        propertyEditors_[5].setTooltip(
            "Coefficient de perte de charge du debit moyen. Son effet acoustique physique est "
            "indirect, via la pression et le debit calcules par le solveur gaz.");
        propertyEditors_[6].setTooltip(
            "Actif dans le reseau physique uniquement pour un resonateur terminal: la longueur "
            "acoustique de reference vaut c/(4f). Zero conserve la longueur geometrique.");
        propertyEditors_[7].setTooltip(
            "Compatibilite du rendu audio de secours. Le guide d'onde physique reste passif et "
            "n'applique pas ce gain arbitraire.");
        updateComponentButton_.setButtonText("METTRE A JOUR LE COMPOSANT");
        updateComponentButton_.onClick = [this] { updateSelectedComponent(); };
        packingDemoButton_.setButtonText("GARNISSAGE DEMO");
        packingDemoButton_.onClick = [this] { setPackingPreset(true); };
        packingBypassButton_.setButtonText("BYPASS GARNISSAGE");
        packingBypassButton_.onClick = [this] { setPackingPreset(false); };
        componentTypeSelector_.onChange = [this] {
            updatePackingEditorAvailability(true);
        };

        connectionTitle_.setText("Connexions composant -> composant", juce::dontSendNotification);
        mappingTitle_.setText("Cylindre -> premier composant", juce::dontSendNotification);
        for (auto* label : std::array<juce::Label*, 3> {
                 &componentTypeLabel_, &connectionTitle_, &mappingTitle_ }) {
            label->setColour(juce::Label::textColourId, juce::Colour(0xffb6c5c0));
            label->setFont(juce::FontOptions(12.0F, juce::Font::bold));
            addAndMakeVisible(*label);
        }
        fromSelector_.setTextWhenNothingSelected("Depuis");
        toSelector_.setTextWhenNothingSelected("Vers");
        cylinderSelector_.setTextWhenNothingSelected("Cylindre");
        mappingComponentSelector_.setTextWhenNothingSelected("Composant");
        addConnectionButton_.setButtonText("CONNECTER");
        addConnectionButton_.onClick = [this] { addConnection(); };
        deleteConnectionButton_.setButtonText("SUPPRIMER");
        deleteConnectionButton_.onClick = [this] { deleteConnection(); };
        assignCylinderButton_.setButtonText("AFFECTER");
        assignCylinderButton_.onClick = [this] { assignCylinder(); };
        deleteMappingButton_.setButtonText("DESAFFECTER");
        deleteMappingButton_.onClick = [this] { deleteMapping(); };

        statusLabel_.setFont(juce::FontOptions(12.0F));
        statusLabel_.setJustificationType(juce::Justification::centredLeft);
        addAndMakeVisible(pathSelector_);
        addAndMakeVisible(newPathButton_);
        addAndMakeVisible(deletePathButton_);
        addAndMakeVisible(legacyButton_);
        addAndMakeVisible(applyButton_);
        addAndMakeVisible(pathCylinderSelector_);
        addAndMakeVisible(destinationPathSelector_);
        addAndMakeVisible(moveCylinderButton_);
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
             type <= static_cast<int>(ExhaustComponentType::outlet); ++type) {
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
            pathSelector_.addItem("Chemin #" + juce::String(path.id) + "  ("
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
                pathCylinderSelector_.addItem("Cylindre " + juce::String(cylinderId),
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
            destinationPathSelector_.addItem("Vers chemin #" + juce::String(candidate.id)
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
            setStatus("Selectionnez un cylindre du chemin courant.", true);
            return;
        }
        const auto result = ExhaustPathTopologyEditor::createPathFromCylinder(
            working_, source->id, *cylinderId);
        if (!result) {
            setStatus("Creation refusee: " + utf8(result.error), true);
            return;
        }
        selectEditedPath(result.selectedPathId);
        setStatus("Nouveau chemin cree; le cylindre a ete transfere atomiquement.", false);
    }

    void moveCylinderToPath() {
        if (!commitSelectedComponentEdits()) return;
        const auto* source = selectedPath();
        const auto cylinderId = selectedPathCylinderId();
        const auto destinationId = selectedDestinationPathId();
        if (source == nullptr || !cylinderId || !destinationId) {
            setStatus("Selectionnez le cylindre et le chemin destination.", true);
            return;
        }
        const auto result = ExhaustPathTopologyEditor::moveCylinder(
            working_, *cylinderId, source->id, *destinationId);
        if (!result) {
            setStatus("Transfert refuse: " + utf8(result.error), true);
            return;
        }
        selectEditedPath(result.selectedPathId);
        setStatus("Cylindre transfere; les racines des graphes ont ete reparees et validees.", false);
    }

    void deletePath() {
        if (!commitSelectedComponentEdits()) return;
        const auto* source = selectedPath();
        const auto destinationId = selectedDestinationPathId();
        if (source == nullptr || !destinationId) {
            setStatus("Choisissez le chemin qui recevra les cylindres avant suppression.", true);
            return;
        }
        const auto result = ExhaustPathTopologyEditor::removePath(
            working_, source->id, *destinationId);
        if (!result) {
            setStatus("Suppression refusee: " + utf8(result.error), true);
            return;
        }
        selectEditedPath(result.selectedPathId);
        setStatus("Chemin supprime; tous ses cylindres ont ete fusionnes vers la destination.", false);
    }

    void rebuildComponentSelectors() {
        const auto preserveFrom = selectedId(fromSelector_);
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
        restoreId(fromSelector_, preserveFrom);
        restoreId(toSelector_, preserveTo);
        restoreId(mappingComponentSelector_, preserveMapping);
    }

    void rebuildCylinderSelector() {
        const auto previousIndex = cylinderSelector_.getSelectedItemIndex();
        cylinderSelector_.clear(juce::dontSendNotification);
        const auto* path = selectedPath();
        if (path == nullptr) return;
        for (std::size_t index = 0; index < path->cylinderIds.size(); ++index)
            cylinderSelector_.addItem("Cylindre " + juce::String(path->cylinderIds[index]),
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
            + "  |  D " + diameter + " mm";
    }

    [[nodiscard]] int connectionCount() const {
        const auto* network = selectedNetwork();
        return network != nullptr ? static_cast<int>(network->connections.size()) : 0;
    }

    [[nodiscard]] juce::String connectionText(int row) const {
        const auto* network = selectedNetwork();
        if (network == nullptr || row < 0 || row >= static_cast<int>(network->connections.size())) return {};
        const auto& connection = network->connections[static_cast<std::size_t>(row)];
        return "#" + juce::String(connection.fromComponentId) + "  ->  #"
            + juce::String(connection.toComponentId);
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
        return "Cylindre " + juce::String(mapping.cylinderId) + "  ->  #"
            + juce::String(mapping.componentId);
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
        const std::array<double, 11> values {
            component->lengthMm, component->diameterMm, component->outletDiameterMm,
            component->volumeLitres,
            component->restriction, component->resonanceHz, component->acousticGain,
            component->dischargeCoefficient,
            component->packingFlowResistivityPaSPerM2,
            component->packingThicknessMm,
            component->perforatedOpenAreaRatio
        };
        propertyEditors_[0].setText(juce::String(component->id), false);
        for (std::size_t index = 0; index < values.size(); ++index)
            propertyEditors_[index + 1].setText(juce::String(values[index], index == 0 ? 1 : 3), false);
        updatePackingEditorAvailability(false);
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
        for (std::size_t index = 9; index < propertyEditors_.size(); ++index) {
            propertyEditors_[index].setEnabled(available);
            if (!available && clearIfUnavailable)
                propertyEditors_[index].setText("0", false);
        }
        const auto tunableBranch = selectedType
            == static_cast<int>(ExhaustComponentType::resonator);
        propertyEditors_[6].setEnabled(tunableBranch);
        if (!tunableBranch && clearIfUnavailable)
            propertyEditors_[6].setText("0", false);
        packingDemoButton_.setEnabled(available);
        packingBypassButton_.setEnabled(available);
    }

    void setPackingPreset(bool enabled) {
        if (componentTypeSelector_.getSelectedItemIndex()
                != static_cast<int>(ExhaustComponentType::muffler)) {
            setStatus("Le garnissage poreux ne s'applique qu'a un silencieux.", true);
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
            setStatus("Impossible de generer un reseau sans cylindre sur ce chemin.", true);
            return;
        }
        path->network = makeEditableExhaustNetwork(*path);
        selectedComponentId_ = path->network->components.front().id;
        selectedConnectionRow_ = -1;
        selectedMappingRow_ = -1;
        setStatus("Reseau simple genere depuis la geometrie legacy. Verifiez puis appliquez.", false);
        rebuildAll();
    }

    [[nodiscard]] std::uint32_t nextComponentId() const {
        const auto* network = selectedNetwork();
        if (network == nullptr || network->components.empty()) return 1;
        std::unordered_set<std::uint32_t> used;
        used.reserve(network->components.size());
        for (const auto& component : network->components) used.insert(component.id);
        for (std::uint64_t candidate = 1;
             candidate <= std::numeric_limits<std::uint32_t>::max(); ++candidate) {
            if (!used.contains(static_cast<std::uint32_t>(candidate)))
                return static_cast<std::uint32_t>(candidate);
        }
        return 0;
    }

    void addComponent() {
        auto* path = selectedPath();
        if (path == nullptr) return;
        if (!path->network) path->network.emplace();
        auto& network = *path->network;
        if (network.components.size() >= maximumComponents) {
            setStatus("Limite atteinte: 256 composants par chemin.", true);
            return;
        }
        const auto typeIndex = addTypeSelector_.getSelectedItemIndex();
        if (typeIndex < 0) {
            setStatus("Selectionnez un type de composant.", true);
            return;
        }
        const auto id = nextComponentId();
        if (id == 0) {
            setStatus("Aucun identifiant de composant disponible.", true);
            return;
        }
        const auto type = static_cast<ExhaustComponentType>(typeIndex);
        network.components.push_back(defaultComponent(type, id));
        selectedComponentId_ = id;
        setStatus(type == ExhaustComponentType::resonator
                ? "Resonateur ajoute: une entree et aucune sortie forment une branche fermee; une sortie le rend inline."
                : "Composant ajoute. Reliez-le avant de valider le reseau.",
            false);
        rebuildAll();
        componentList_.selectRow(static_cast<int>(network.components.size()) - 1);
    }

    void deleteComponent() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedComponentId_ == 0) return;
        const auto id = selectedComponentId_;
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
        setStatus("Composant et references associees supprimes.", false);
        rebuildAll();
    }

    void updateSelectedComponent() {
        auto* component = selectedComponent();
        auto* network = selectedNetwork();
        if (component == nullptr || network == nullptr) return;

        std::uint32_t newId = 0;
        if (!parseId(propertyEditors_[0].getText(), newId)) {
            setStatus("L'ID doit etre un entier unique compris entre 1 et 4294967295.", true);
            return;
        }
        if (newId != component->id && std::any_of(network->components.begin(), network->components.end(),
                [newId](const auto& candidate) { return candidate.id == newId; })) {
            setStatus("Cet identifiant est deja utilise dans le chemin.", true);
            return;
        }
        std::array<double, 11> values {};
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!parseFinite(propertyEditors_[index + 1].getText(), values[index])) {
                setStatus("Toutes les proprietes doivent etre des nombres finis.", true);
                return;
            }
        }
        const auto typeIndex = componentTypeSelector_.getSelectedItemIndex();
        if (typeIndex < static_cast<int>(ExhaustComponentType::pipe)
            || typeIndex > static_cast<int>(ExhaustComponentType::outlet)) {
            setStatus("Type de composant invalide.", true);
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
            || values[5] < 0.0 || values[5] > 20'000.0
            || values[6] < 0.0 || values[6] > 8.0
            || values[7] < 0.02 || values[7] > 1.5
            || values[8] < 0.0 || values[8] > 200'000.0
            || values[9] < 0.0 || values[9] > 300.0
            || values[10] < 0.0 || values[10] > 1.0) {
            setStatus("Valeurs hors limites (L 0..10000, D entree 5..500, D sortie 0 ou 5..500, V 0..1000, restriction 0..20, "
                      "resonance 0..20000, gain 0..8, Cd 0.02..1.5).", true);
            return;
        }
        const auto hasNoPacking = values[8] == 0.0
            && values[9] == 0.0 && values[10] == 0.0;
        const auto hasCompletePacking = type == ExhaustComponentType::muffler
            && values[8] > 0.0 && values[9] > 0.0 && values[10] > 0.0;
        if (!hasNoPacking && !hasCompletePacking) {
            setStatus("Garnissage: les trois valeurs doivent etre nulles, ou positives ensemble sur un silencieux.", true);
            return;
        }

        const auto oldId = component->id;
        component->id = newId;
        component->type = type;
        component->lengthMm = values[0];
        component->diameterMm = values[1];
        component->outletDiameterMm = values[2];
        component->volumeLitres = values[3];
        component->restriction = values[4];
        component->resonanceHz = values[5];
        component->acousticGain = values[6];
        component->dischargeCoefficient = values[7];
        component->packingFlowResistivityPaSPerM2 = values[8];
        component->packingThicknessMm = values[9];
        component->perforatedOpenAreaRatio = values[10];
        if (newId != oldId) {
            for (auto& connection : network->connections) {
                if (connection.fromComponentId == oldId) connection.fromComponentId = newId;
                if (connection.toComponentId == oldId) connection.toComponentId = newId;
            }
            for (auto& mapping : network->cylinderConnections)
                if (mapping.componentId == oldId) mapping.componentId = newId;
        }
        selectedComponentId_ = newId;
        setStatus("Composant mis a jour dans la copie de travail.", false);
        rebuildAll();
    }

    void addConnection() {
        auto* network = selectedNetwork();
        const auto from = selectedId(fromSelector_);
        const auto to = selectedId(toSelector_);
        if (network == nullptr || !from || !to) {
            setStatus("Selectionnez un composant source et un composant destination.", true);
            return;
        }
        if (*from == *to) {
            setStatus("Un composant ne peut pas etre relie a lui-meme.", true);
            return;
        }
        if (network->connections.size() >= maximumConnections) {
            setStatus("Limite atteinte: 1024 connexions par chemin.", true);
            return;
        }
        if (std::any_of(network->connections.begin(), network->connections.end(),
                [from, to](const auto& connection) {
                    return connection.fromComponentId == *from && connection.toComponentId == *to;
                })) {
            setStatus("Cette connexion existe deja.", true);
            return;
        }
        network->connections.push_back({ *from, *to });
        selectedConnectionRow_ = static_cast<int>(network->connections.size()) - 1;
        setStatus("Connexion ajoutee. La validation detectera cycles et cardinalites invalides.", false);
        rebuildAll();
        connectionList_.selectRow(selectedConnectionRow_);
    }

    void deleteConnection() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedConnectionRow_ < 0
            || selectedConnectionRow_ >= static_cast<int>(network->connections.size())) {
            setStatus("Selectionnez une connexion a supprimer.", true);
            return;
        }
        network->connections.erase(network->connections.begin() + selectedConnectionRow_);
        selectedConnectionRow_ = -1;
        setStatus("Connexion supprimee.", false);
        rebuildAll();
    }

    void assignCylinder() {
        auto* network = selectedNetwork();
        const auto* path = selectedPath();
        const auto componentId = selectedId(mappingComponentSelector_);
        const auto cylinderIndex = cylinderSelector_.getSelectedItemIndex();
        if (network == nullptr || path == nullptr || !componentId || cylinderIndex < 0
            || cylinderIndex >= static_cast<int>(path->cylinderIds.size())) {
            setStatus("Selectionnez un cylindre et son premier composant.", true);
            return;
        }
        const auto cylinderId = path->cylinderIds[static_cast<std::size_t>(cylinderIndex)];
        const auto existing = std::find_if(network->cylinderConnections.begin(),
            network->cylinderConnections.end(), [cylinderId](const auto& mapping) {
                return mapping.cylinderId == cylinderId;
            });
        if (existing != network->cylinderConnections.end()) {
            existing->componentId = *componentId;
            selectedMappingRow_ = static_cast<int>(std::distance(network->cylinderConnections.begin(), existing));
        } else {
            network->cylinderConnections.push_back({ cylinderId, *componentId });
            selectedMappingRow_ = static_cast<int>(network->cylinderConnections.size()) - 1;
        }
        setStatus("Affectation cylindre mise a jour.", false);
        rebuildAll();
        mappingList_.selectRow(selectedMappingRow_);
    }

    void deleteMapping() {
        auto* network = selectedNetwork();
        if (network == nullptr || selectedMappingRow_ < 0
            || selectedMappingRow_ >= static_cast<int>(network->cylinderConnections.size())) {
            setStatus("Selectionnez une affectation a supprimer.", true);
            return;
        }
        network->cylinderConnections.erase(network->cylinderConnections.begin() + selectedMappingRow_);
        selectedMappingRow_ = -1;
        setStatus("Affectation supprimee; le cylindre devra etre reaffecte avant validation.", false);
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
        const auto appliedM = shortestExhaustCellM(working_);
        const auto baselineM = shortestExhaustCellM(baseline_);
        if (!(appliedM > 0.0)) return {};
        auto text = "  Maille la plus fine " + juce::String(appliedM * 1'000.0, 1)
            + " mm";
        if (!(baselineM > 0.0)) return text + ".";
        // Rate is proportional to 1/dx for the same gas, so this ratio is an
        // exact ratio of solver cost with nothing assumed about the gas state.
        const auto costRatio = baselineM / appliedM;
        text += " contre " + juce::String(baselineM * 1'000.0, 1)
            + " mm auparavant";
        if (costRatio >= 1.5) {
            text += " : le solveur d'echappement sous-cadencera "
                + juce::String(costRatio, 1)
                + "x plus vite. Verifiez le facteur temps reel;"
                  " un element court impose son pas a TOUT le reseau.";
        } else if (costRatio <= 0.67) {
            text += " : cout du solveur divise par "
                + juce::String(1.0 / costRatio, 1) + ".";
        } else {
            text += " (cout du solveur inchange).";
        }
        return text;
    }

    void apply() {
        if (selectedComponent() != nullptr) {
            updateSelectedComponent();
            if (statusIsError_) return;
        }
        if (const auto error = validateEngineConfig(working_)) {
            setStatus("Configuration refusee: " + utf8(*error), true);
            juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon,
                "Reseau d'echappement invalide", utf8(*error));
            return;
        }
        if (!applyCallback_) {
            setStatus("Configuration valide, mais aucun callback d'application n'est installe.", true);
            return;
        }
        try {
            if (!applyCallback_(working_)) {
                setStatus("La configuration valide a ete refusee par le moteur actif.", true);
                return;
            }
            setStatus("Configuration validee et appliquee au moteur."
                + solverCostAdvisory(), false);
        } catch (const std::exception& exception) {
            setStatus("L'application a echoue: " + utf8(exception.what()), true);
        } catch (...) {
            setStatus("L'application a echoue avec une erreur inconnue.", true);
        }
    }

    void setStatus(const juce::String& message, bool error) {
        statusIsError_ = error;
        statusLabel_.setText((error ? "ERREUR  |  " : "INFO  |  ") + message,
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
    juce::TextButton applyButton_;
    juce::ComboBox pathCylinderSelector_;
    juce::ComboBox destinationPathSelector_;
    juce::TextButton moveCylinderButton_;
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
    : juce::DocumentWindow("EngineLab - Concepteur d'echappement", juce::Colour(0xff0d1312),
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

void ExhaustDesignerWindow::setConfig(const EngineConfig& config) {
    if (content_ != nullptr) content_->setConfig(config);
}

} // namespace enginelab
