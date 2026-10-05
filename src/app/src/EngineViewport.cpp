#include <enginelab/app/EngineViewport.hpp>
#include <enginelab/exhaust/ExhaustComponentResize.hpp>
#include <enginelab/foundation/CylinderResize.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

namespace enginelab::ui {
namespace {
[[nodiscard]] double crankThrowMmFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    if (cylinder.crankJournalId != 0) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& item) { return item.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()) return journal->throwMm;
    }
    return cylinder.strokeMm * 0.5;
}

[[nodiscard]] double crankOffsetDegreesFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    (void)config;
    return cylinder.crankOffsetDegrees;
}

[[nodiscard]] double mechanicalCrankOffsetDegreesFor(const EngineConfig& config,
                                                      const CylinderConfig& cylinder) noexcept {
    if (config.layout == EngineLayout::radial) return cylinder.bankOffsetDegrees;
    if (cylinder.crankJournalId != 0) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& item) { return item.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()) return journal->angleDegrees;
    }
    return cylinder.crankOffsetDegrees;
}
} // namespace

EngineCutawayView::EngineCutawayView(const DashboardModel& model) : model_(model) {
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
}

void EngineCutawayView::setLayer(int layer) {
    layer_ = std::clamp(layer, 0, 3);
    repaint();
}

void EngineCutawayView::paint(juce::Graphics& g) {
    const auto area = getLocalBounds().toFloat().reduced(24.0F, 0.0F).withTrimmedTop(56.0F)
        .withTrimmedBottom(40.0F);
    const auto centre = area.getCentre();
    g.addTransform(juce::AffineTransform::translation(-centre.x, -centre.y)
        .scaled(zoom_)
        .translated(centre.x + pan_.x, centre.y + pan_.y));
    drawEngine(g, area);
}

void EngineCutawayView::mouseWheelMove(const juce::MouseEvent& event, const juce::MouseWheelDetails& wheel) {
    if (wheelModifierActive && wheelModifierActive()) {
        juce::Component::mouseWheelMove(event, wheel);
        return;
    }
    zoom_ = std::clamp(zoom_ * std::exp(wheel.deltaY * 0.55F), 0.45F, 3.5F);
    repaint();
}

void EngineCutawayView::mouseDown(const juce::MouseEvent&) { dragStartPan_ = pan_; }

void EngineCutawayView::mouseDrag(const juce::MouseEvent& event) {
    pan_ = dragStartPan_ + event.getOffsetFromDragStart().toFloat() / zoom_;
    repaint();
}

void EngineCutawayView::mouseDoubleClick(const juce::MouseEvent&) {
    pan_ = {};
    zoom_ = 1.0F;
    repaint();
}

void EngineCutawayView::drawEngine(juce::Graphics& g, juce::Rectangle<float> area) const {
    const auto& config = model_.config;
    const auto& state = model_.state;
    const auto count = static_cast<int>(config.cylinders.size());
    if (count <= 0) return;
    const auto showFlow = layer_ == 0 || layer_ == 3;
    const auto showCombustion = layer_ == 0 || layer_ == 1;
    const auto showValvetrain = layer_ == 0 || layer_ == 1;
    if (config.layout == EngineLayout::radial) {
        auto body = area.reduced(10.0F);
        const auto diameter = std::min(body.getWidth(), body.getHeight());
        const auto radius = diameter * 0.30F;
        const auto strokeTravel = diameter * 0.13F;
        const auto cylinderWidth = std::clamp(diameter * 0.115F, 42.0F, 72.0F);
        const auto cylinderHeight = std::clamp(diameter * 0.245F, 88.0F, 138.0F);
        const auto center = body.getCentre();
        g.setColour(juce::Colour(0xff18221f));
        g.fillEllipse(center.x - radius * 0.72F, center.y - radius * 0.72F, radius * 1.44F, radius * 1.44F);
        g.setColour(juce::Colour(0xff2b3834));
        g.drawEllipse(center.x - radius * 0.72F, center.y - radius * 0.72F, radius * 1.44F, radius * 1.44F, 1.4F);
        g.setColour(juce::Colour(0x3341b6d7));
        g.drawEllipse(center.x - radius * 1.13F, center.y - radius * 1.13F, radius * 2.26F, radius * 2.26F, 5.0F);
        g.setColour(juce::Colour(0x44ef6f3c));
        g.drawEllipse(center.x - radius * 1.34F, center.y - radius * 1.34F, radius * 2.68F, radius * 2.68F, 5.0F);

        for (int index = 0; index < count; ++index) {
            const auto& cylinder = config.cylinders[static_cast<std::size_t>(index)];
            const auto* liveCylinder = static_cast<std::size_t>(index) < state.cylinderStateCount
                ? &state.cylinderStates[static_cast<std::size_t>(index)] : nullptr;
            const auto mechanicalPhase = std::fmod(state.crankAngleDegrees
                - mechanicalCrankOffsetDegreesFor(config, cylinder) + 720.0, 360.0);
            const auto pistonAngle = mechanicalPhase * std::numbers::pi / 180.0;
            const auto crankRadius = crankThrowMmFor(config, cylinder);
            const auto rodLength = std::max(cylinder.connectingRodMm, crankRadius + 0.1);
            const auto sliderTravel = crankRadius * (1.0 - std::cos(pistonAngle)) + rodLength
                - std::sqrt(std::max(0.0, rodLength * rodLength
                    - crankRadius * crankRadius * std::sin(pistonAngle) * std::sin(pistonAngle)));
            const auto travel = static_cast<float>(std::clamp(liveCylinder != nullptr
                ? liveCylinder->pistonTravelMm / cylinder.strokeMm : sliderTravel / cylinder.strokeMm, 0.0, 1.0));
            const auto angle = -std::numbers::pi * 0.5
                + cylinder.bankOffsetDegrees * std::numbers::pi / 180.0;
            const auto unitX = static_cast<float>(std::cos(angle));
            const auto unitY = static_cast<float>(std::sin(angle));
            // The barrel belongs to the crankcase and must remain fixed. Only
            // the piston/wrist pin travels on the cylinder axis. The previous
            // code reused the live wrist-pin point as the chamber centre, which
            // made the complete radial cylinder follow its connecting rod.
            const auto cylinderCenterX = center.x
                + unitX * (radius + strokeTravel * 0.5F);
            const auto cylinderCenterY = center.y
                + unitY * (radius + strokeTravel * 0.5F);
            auto pistonCenterX = center.x + unitX * (radius + travel * strokeTravel);
            auto pistonCenterY = center.y + unitY * (radius + travel * strokeTravel);
            const auto sharedCrankAngle = state.crankAngleDegrees * std::numbers::pi / 180.0;
            auto crankPinX = center.x + static_cast<float>(std::cos(sharedCrankAngle)) * diameter * 0.060F;
            auto crankPinY = center.y + static_cast<float>(std::sin(sharedCrankAngle)) * diameter * 0.060F;
            if (liveCylinder != nullptr) {
                const auto mechanicalScale = radius / static_cast<float>(rodLength + crankRadius);
                crankPinX = center.x + static_cast<float>(liveCylinder->crankPinXMm) * mechanicalScale;
                crankPinY = center.y + static_cast<float>(liveCylinder->crankPinYMm) * mechanicalScale;
                pistonCenterX = center.x + static_cast<float>(liveCylinder->wristPinXMm) * mechanicalScale;
                pistonCenterY = center.y + static_cast<float>(liveCylinder->wristPinYMm) * mechanicalScale;
            }
            if (showFlow && liveCylinder != nullptr) {
                const auto intakePulse = static_cast<float>(std::clamp(liveCylinder->intakeFlowMgPerCycle / 55.0, 0.0, 1.0));
                const auto exhaustPulse = static_cast<float>(std::clamp(liveCylinder->exhaustFlowMgPerCycle / 55.0, 0.0, 1.0));
                g.setColour(juce::Colour(0xff41b6d7).withAlpha(0.18F + intakePulse * 0.52F));
                g.drawLine(center.x + unitX * radius * 1.13F, center.y + unitY * radius * 1.13F,
                           cylinderCenterX, cylinderCenterY, 1.4F + intakePulse * 3.4F);
                g.setColour(juce::Colour(0xffef6f3c).withAlpha(0.18F + exhaustPulse * 0.52F));
                g.drawLine(center.x + unitX * radius * 1.34F, center.y + unitY * radius * 1.34F,
                           cylinderCenterX, cylinderCenterY, 1.4F + exhaustPulse * 3.4F);
            }
            g.setColour(juce::Colour(0xff9ba8a3));
            g.drawLine(pistonCenterX, pistonCenterY, crankPinX, crankPinY, 4.0F);
            g.saveState();
            g.addTransform(juce::AffineTransform::rotation(static_cast<float>(angle + std::numbers::pi * 0.5),
                                                           cylinderCenterX, cylinderCenterY));
            const auto chamber = juce::Rectangle<float>(cylinderCenterX - cylinderWidth * 0.5F,
                                                        cylinderCenterY - cylinderHeight * 0.5F,
                                                        cylinderWidth, cylinderHeight);
            g.setColour(juce::Colour(0xff111817));
            g.fillRoundedRectangle(chamber, 7.0F);
            g.setColour(juce::Colour(0xff3a4743));
            g.drawRoundedRectangle(chamber, 7.0F, 1.3F);
            if (showCombustion && liveCylinder != nullptr && liveCylinder->combustionActive && liveCylinder->combustionPulse > 0.08) {
                g.setColour(liveCylinder->misfiring ? juce::Colour(0x88ffca28) : juce::Colour(0x99ff7a3d));
                g.fillEllipse(chamber.reduced(8.0F, cylinderHeight * 0.35F));
            }
            g.setColour(juce::Colour(0xff41b6d7));
            g.drawLine(chamber.getX() + cylinderWidth * 0.30F, chamber.getY() + 5.0F,
                       chamber.getX() + cylinderWidth * 0.30F, chamber.getY() + 23.0F, 3.0F);
            g.setColour(juce::Colour(0xffef6f3c));
            g.drawLine(chamber.getX() + cylinderWidth * 0.70F, chamber.getY() + 5.0F,
                       chamber.getX() + cylinderWidth * 0.70F, chamber.getY() + 23.0F, 3.0F);
            g.setColour(juce::Colour(0xffb7c1bd));
            g.fillRoundedRectangle(chamber.getX() + 5.0F, chamber.getBottom() - 28.0F - travel * 18.0F,
                                   cylinderWidth - 10.0F, 20.0F, 3.0F);
            g.restoreState();
            if (liveCylinder != nullptr) {
                g.setColour(liveCylinder->misfireProbability > 0.2 ? juce::Colour(0xffffca28) : juce::Colour(0xffaab7b2));
                g.setFont(juce::FontOptions(10.5F, juce::Font::bold));
                g.drawFittedText("C" + juce::String(liveCylinder->id) + "  "
                    + juce::String(liveCylinder->pressureEstimateBar, 1) + " bar",
                    juce::Rectangle<float>(cylinderCenterX - 38.0F, cylinderCenterY - 8.0F, 76.0F, 16.0F).toNearestInt(),
                    juce::Justification::centred, 1);
            }
        }
        const auto crankNeedle = state.crankAngleDegrees * std::numbers::pi / 180.0;
        g.setColour(juce::Colour(0xffef6f3c));
        g.fillEllipse(center.x - 11.0F, center.y - 11.0F, 22.0F, 22.0F);
        g.drawLine(center.x, center.y,
                   center.x + static_cast<float>(std::cos(crankNeedle)) * diameter * 0.085F,
                   center.y + static_cast<float>(std::sin(crankNeedle)) * diameter * 0.085F, 4.0F);
        return;
    }
    const auto bankCount = std::max<std::size_t>(1, config.banks.size());
    int columns = config.banks.empty() ? count : 1;
    for (const auto& bank : config.banks)
        columns = std::max(columns, static_cast<int>(bank.cylinderIds.size()));
    const auto spacing = 14.0F;
    const auto cylinderWidth = std::clamp((area.getWidth() - spacing * static_cast<float>(columns - 1)) / static_cast<float>(columns), 46.0F, 112.0F);
    const auto totalWidth = cylinderWidth * static_cast<float>(columns) + spacing * static_cast<float>(columns - 1);
    const auto startX = area.getCentreX() - totalWidth * 0.5F;
    const auto bankGap = bankCount > 1 ? 14.0F : 0.0F;
    const auto bankHeight = bankCount > 1
        ? std::max(54.0F, (area.getHeight() * 0.90F - bankGap * static_cast<float>(bankCount - 1))
            / static_cast<float>(bankCount))
        : area.getHeight() * 0.72F;

    const auto runnerTop = area.getY() + 10.0F;
    g.setColour(juce::Colour(0x3341b6d7));
    g.fillRoundedRectangle(area.withY(runnerTop).withHeight(11.0F).reduced(12.0F, 0.0F), 5.0F);
    g.setColour(juce::Colour(0x44ef6f3c));
    g.fillRoundedRectangle(area.withY(area.getBottom() - 18.0F).withHeight(11.0F).reduced(12.0F, 0.0F), 5.0F);

    for (int index = 0; index < count; ++index) {
        const auto& cylinder = config.cylinders[static_cast<std::size_t>(index)];
        const auto* liveCylinder = static_cast<std::size_t>(index) < state.cylinderStateCount
            ? &state.cylinderStates[static_cast<std::size_t>(index)] : nullptr;
        std::size_t bankIndex = 0;
        int column = index;
        const CylinderBankConfig* bankConfig = nullptr;
        for (std::size_t candidate = 0; candidate < config.banks.size(); ++candidate) {
            const auto found = std::find(config.banks[candidate].cylinderIds.begin(),
                                         config.banks[candidate].cylinderIds.end(), cylinder.id);
            if (found != config.banks[candidate].cylinderIds.end()) {
                bankIndex = candidate;
                column = static_cast<int>(std::distance(config.banks[candidate].cylinderIds.begin(), found));
                bankConfig = &config.banks[candidate];
                break;
            }
        }
        const auto phase = std::fmod(state.crankAngleDegrees
            - crankOffsetDegreesFor(config, cylinder) + 720.0, 720.0);
        const auto mechanicalPhase = std::fmod(state.crankAngleDegrees
            - mechanicalCrankOffsetDegreesFor(config, cylinder) + 720.0, 360.0);
        const auto pistonAngle = mechanicalPhase * std::numbers::pi / 180.0;
        const auto crankRadius = crankThrowMmFor(config, cylinder);
        const auto rodLength = std::max(cylinder.connectingRodMm, crankRadius + 0.1);
        const auto sliderTravel = crankRadius * (1.0 - std::cos(pistonAngle)) + rodLength
            - std::sqrt(std::max(0.0, rodLength * rodLength
                - crankRadius * crankRadius * std::sin(pistonAngle) * std::sin(pistonAngle)));
        const auto travel = static_cast<float>(std::clamp(liveCylinder != nullptr
            ? liveCylinder->pistonTravelMm / cylinder.strokeMm : sliderTravel / cylinder.strokeMm, 0.0, 1.0));
        const auto x = startX + static_cast<float>(column) * (cylinderWidth + spacing);
        const auto top = area.getY() + 20.0F + static_cast<float>(bankIndex) * (bankHeight + bankGap);
        const auto height = bankHeight - 18.0F;
        const auto pistonY = top + 43.0F + travel * (height - 92.0F);
        const auto centerX = x + cylinderWidth * 0.5F;
        const auto crankY = top + height - 18.0F;
        g.saveState();
        if (bankConfig != nullptr && std::abs(bankConfig->angleDegrees) > 0.1) {
            const auto visualDegrees = std::clamp(bankConfig->angleDegrees * 0.12, -12.0, 12.0);
            g.addTransform(juce::AffineTransform::rotation(static_cast<float>(visualDegrees * std::numbers::pi / 180.0),
                                                           centerX, crankY));
        }
        g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(x, top, cylinderWidth, height, 7.0F);
        g.setColour(juce::Colour(0xff3a4743)); g.drawRoundedRectangle(x, top, cylinderWidth, height, 7.0F, 1.5F);
        const auto fallbackIntakeLift = profiledValveLiftMm(phase, 360.0 + config.camshafts.intakeCenterlineDegrees,
            config.camshafts.intakeDurationDegrees, config.camshafts.intakeLiftMm, config.camshafts.intakeLiftProfile);
        const auto fallbackExhaustLift = profiledValveLiftMm(phase, 360.0 - config.camshafts.exhaustCenterlineDegrees,
            config.camshafts.exhaustDurationDegrees, config.camshafts.exhaustLiftMm, config.camshafts.exhaustLiftProfile);
        const auto intakeLift = liveCylinder != nullptr ? liveCylinder->intakeValveLiftMm : fallbackIntakeLift;
        const auto exhaustLift = liveCylinder != nullptr ? liveCylinder->exhaustValveLiftMm : fallbackExhaustLift;
        const auto intakeDrop = static_cast<float>(intakeLift
            / std::max({ 1.0, config.camshafts.intakeLiftMm, config.camshafts.highIntakeLiftMm }) * 17.0);
        const auto exhaustDrop = static_cast<float>(exhaustLift
            / std::max({ 1.0, config.camshafts.exhaustLiftMm, config.camshafts.highExhaustLiftMm }) * 17.0);
        if (showValvetrain) {
            g.setColour(juce::Colour(0xff41b6d7)); g.drawLine(x + cylinderWidth * 0.30F, top + 5.0F, x + cylinderWidth * 0.30F, top + 22.0F + intakeDrop, 3.0F);
            g.fillEllipse(x + cylinderWidth * 0.21F, top + 18.0F + intakeDrop, cylinderWidth * 0.18F, 4.0F);
            g.setColour(juce::Colour(0xffef6f3c)); g.drawLine(x + cylinderWidth * 0.70F, top + 5.0F, x + cylinderWidth * 0.70F, top + 22.0F + exhaustDrop, 3.0F);
            g.fillEllipse(x + cylinderWidth * 0.61F, top + 18.0F + exhaustDrop, cylinderWidth * 0.18F, 4.0F);
        }
        if (showCombustion && liveCylinder != nullptr && liveCylinder->combustionActive && liveCylinder->combustionPulse > 0.08) {
            g.setColour(liveCylinder->misfiring ? juce::Colour(0x88ffca28) : juce::Colour(0x99ff7a3d));
            g.fillEllipse(x + 9.0F, top + 30.0F, cylinderWidth - 18.0F, 22.0F);
        }
        if (showFlow && liveCylinder != nullptr) {
            const auto intakePulse = static_cast<float>(std::clamp(liveCylinder->intakeFlowMgPerCycle / 45.0, 0.0, 1.0));
            const auto exhaustPulse = static_cast<float>(std::clamp(liveCylinder->exhaustFlowMgPerCycle / 45.0, 0.0, 1.0));
            g.setColour(juce::Colour(0xff41b6d7).withAlpha(0.20F + intakePulse * 0.58F));
            g.drawLine(centerX, runnerTop + 9.0F, centerX, top + 25.0F, 2.0F + intakePulse * 4.0F);
            g.setColour(juce::Colour(0xffef6f3c).withAlpha(0.20F + exhaustPulse * 0.58F));
            g.drawLine(centerX, top + height - 24.0F, centerX, area.getBottom() - 13.0F, 2.0F + exhaustPulse * 4.0F);
        }
        g.setColour(juce::Colour(0xffb7c1bd)); g.fillRoundedRectangle(x + 5.0F, pistonY, cylinderWidth - 10.0F, 22.0F, 3.0F);
        const auto crankX = centerX + static_cast<float>(std::sin(pistonAngle)) * 14.0F;
        g.setColour(juce::Colour(0xff9ba8a3)); g.drawLine(centerX, pistonY + 18.0F, crankX, crankY, 4.0F);
        g.setColour(juce::Colour(0xffef6f3c)); g.fillEllipse(crankX - 4.0F, crankY - 4.0F, 8.0F, 8.0F);
        if (liveCylinder != nullptr) {
            const auto& cylinderState = *liveCylinder;
            g.setColour(cylinderState.misfireProbability > 0.2 ? juce::Colour(0xffffca28) : juce::Colour(0xff82918b));
            g.setFont(juce::FontOptions(10.5F, juce::Font::bold));
            g.drawFittedText("C" + juce::String(cylinderState.id) + "  "
                + juce::String(cylinderState.pressureEstimateBar, 1) + " bar",
                juce::Rectangle<float>(x + 3.0F, top + height - 14.0F, cylinderWidth - 6.0F, 12.0F).toNearestInt(),
                juce::Justification::centred, 1);
        }
        g.restoreState();
    }
}


// ------------------------------------------------------------------ settings
juce::File ViewSettings::file() {
    return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
        .getChildFile("EngineLab").getChildFile("view.json");
}

void ViewSettings::load() {
    const auto parsed = juce::JSON::parse(file());
    const auto* object = parsed.getDynamicObject();
    if (object == nullptr) return;
    const auto flag = [object](const char* name, bool fallback) {
        return object->hasProperty(name) ? static_cast<bool>(object->getProperty(name)) : fallback;
    };
    renderer3d = object->getProperty("renderer").toString() != "2d";
    if (object->hasProperty("frameRateCap"))
        frameRateCap = std::clamp(static_cast<int>(object->getProperty("frameRateCap")), 0, 1000);
    vsync = flag("vsync", vsync);
    motionBlur = flag("motionBlur", motionBlur);
    antiAliasing = flag("antiAliasing", antiAliasing);
}

void ViewSettings::save() const {
    auto object = std::make_unique<juce::DynamicObject>();
    object->setProperty("renderer", renderer3d ? "3d" : "2d");
    object->setProperty("frameRateCap", frameRateCap);
    object->setProperty("vsync", vsync);
    object->setProperty("motionBlur", motionBlur);
    object->setProperty("antiAliasing", antiAliasing);
    const auto target = file();
    (void)target.getParentDirectory().createDirectory();
    (void)target.replaceWithText(juce::JSON::toString(juce::var(object.release())));
}

// --------------------------------------------------------------- cycle panel
namespace {
constexpr int cycleRowHeight = 17;
constexpr std::size_t cycleMaximumRows = 8;
const std::array<const char*, 4> strokeNames { "Power", "Exhaust", "Intake", "Compression" };
const std::array<juce::Colour, 4> strokeColours { juce::Colour(0xffef6f3c), juce::Colour(0xffc9a493),
                                                  juce::Colour(0xff41b6d7), juce::Colour(0xffa9b8b3) };
const std::array<juce::Colour, 4> strokeBands { juce::Colour(0x55ef6f3c), juce::Colour(0x448a6a5a),
                                                juce::Colour(0x4441b6d7), juce::Colour(0x446a7a75) };
} // namespace

void CyclePanel::setCylinders(std::vector<juce::String> labels) {
    labels_ = std::move(labels);
    repaint();
}

int CyclePanel::idealHeight() const {
    const auto rows = std::min(labels_.size(), cycleMaximumRows) + (labels_.size() > cycleMaximumRows ? 1U : 0U);
    return static_cast<int>(rows) * cycleRowHeight + 16;
}

void CyclePanel::paint(juce::Graphics& g) {
    const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
    g.setColour(juce::Colour(0xd10d1211));
    g.fillRoundedRectangle(bounds, 9.0F);
    g.setColour(colours::lineStrong);
    g.drawRoundedRectangle(bounds, 9.0F, 1.0F);
    if (!phaseOf) return;
    auto area = getLocalBounds().reduced(12, 8);
    const auto rows = std::min(labels_.size(), cycleMaximumRows);
    for (std::size_t i = 0; i < rows; ++i) {
        auto row = area.removeFromTop(cycleRowHeight).toFloat();
        const auto phase = std::clamp(phaseOf(i), 0.0, 719.999);
        const auto stroke = static_cast<std::size_t>(phase / 180.0);
        g.setFont(monoFont(11.0F));
        g.setColour(colours::muted);
        g.drawText(labels_[i], row.removeFromLeft(28.0F), juce::Justification::centredLeft, false);
        g.setColour(strokeColours[stroke]);
        g.drawText(strokeNames[stroke], row.removeFromRight(78.0F), juce::Justification::centredRight, false);
        const auto track = row.withSizeKeepingCentre(row.getWidth() - 8.0F, 6.0F);
        juce::Path clip;
        clip.addRoundedRectangle(track, 3.0F);
        g.saveState();
        g.reduceClipRegion(clip);
        for (std::size_t band = 0; band < 4; ++band) {
            g.setColour(strokeBands[band]);
            g.fillRect(track.withX(track.getX() + track.getWidth() * static_cast<float>(band) * 0.25F)
                           .withWidth(track.getWidth() * 0.25F));
        }
        g.restoreState();
        g.setColour(juce::Colours::white);
        g.fillRect(juce::Rectangle<float>(track.getX() + track.getWidth() * static_cast<float>(phase / 720.0) - 1.0F,
                                          track.getY() - 2.0F, 2.0F, 10.0F));
    }
    if (labels_.size() > cycleMaximumRows) {
        g.setFont(uiFont(11.5F));
        g.setColour(colours::faint);
        g.drawText("+" + juce::String(static_cast<int>(labels_.size() - cycleMaximumRows)) + " more cylinders",
                   area.removeFromTop(cycleRowHeight), juce::Justification::centredLeft, false);
    }
}

// --------------------------------------------------------------- wave legend
namespace {
/** The scale as the legend prints it. juce::String(float, 0) means "default
    precision", not "no decimals". */
juce::String legendFigure(float pascals) {
    return pascals < 10'000.0F ? juce::String(pascals / 1'000.0F, 1) : juce::String(juce::roundToInt(pascals / 1'000.0F));
}
} // namespace

void WaveLegend::setScale(float pascals) {
    // Repaint only when the shown figure changes.
    if (legendFigure(pascals) == legendFigure(scalePa_)) return;
    scalePa_ = pascals;
    repaint();
}

void WaveLegend::paint(juce::Graphics& g) {
    auto area = getLocalBounds().toFloat();
    g.setColour(juce::Colour(0xd10d1211));
    g.fillRoundedRectangle(area, 9.0F);
    g.setColour(colours::lineStrong);
    g.drawRoundedRectangle(area.reduced(0.5F), 9.0F, 1.0F);
    area = area.reduced(12.0F, 8.0F);

    auto header = area.removeFromTop(14.0F);
    g.setFont(uiFont(10.5F, true).withExtraKerningFactor(0.06F));
    g.setColour(colours::muted);
    g.drawText(utf8("PRESSURE WAVES"), header, juce::Justification::centredLeft, false);
    g.setFont(monoFont(10.5F, false));
    g.drawText(utf8("±") + legendFigure(scalePa_) + " kPa", header,
               juce::Justification::centredRight, false);

    // The bar of the prototype: violet, grey at the mean, orange, pale yellow.
    const auto bar = area.removeFromTop(14.0F).withSizeKeepingCentre(area.getWidth(), 8.0F);
    juce::ColourGradient gradient(juce::Colour(0xff9a7bff), bar.getX(), 0.0F, juce::Colour(0xffffe6b0), bar.getRight(),
                                  0.0F, false);
    gradient.addColour(0.5, juce::Colour(0xff4a5451));
    gradient.addColour(0.85, juce::Colour(0xffff6a2b));
    g.setGradientFill(gradient);
    g.fillRoundedRectangle(bar, 4.0F);

    g.setFont(uiFont(11.0F));
    g.setColour(colours::muted);
    const auto ticks = area.removeFromTop(14.0F);
    g.drawText(utf8("below mean"), ticks, juce::Justification::centredLeft, false);
    g.drawText(utf8("cycle mean"), ticks, juce::Justification::centred, false);
    g.drawText(utf8("above mean"), ticks, juce::Justification::centredRight, false);
}

// ------------------------------------------------------------ part inspector
namespace {
constexpr int inspectorRowHeight = 19;
constexpr int inspectorHeader = 46;
constexpr int inspectorScopeHeight = 118;
constexpr int inspectorEditRow = 26;
constexpr int inspectorEditButtons = 30;
constexpr int inspectorEditHeader = 24;
constexpr int inspectorStepperWidth = 26;
constexpr int inspectorValueWidth = 78;
} // namespace

PartInspector::PartInspector() {
    for (auto* button : { &firstDown_, &firstUp_, &secondDown_, &secondUp_, &resetEdit_, &rampEdit_ })
        addChildComponent(button);
    firstDown_.onClick = [this] { stepEdit(true, -1.0); };
    firstUp_.onClick = [this] { stepEdit(true, 1.0); };
    secondDown_.onClick = [this] { stepEdit(false, -1.0); };
    secondUp_.onClick = [this] { stepEdit(false, 1.0); };
    rampEdit_.setTooltip("When the cylinders take a new size: at their next gas-exchange TDC, or gradually over "
                         + juce::String(EngineViewport::cylinderRampSeconds, 0) + " s");
    rampEdit_.onClick = [this] {
        gradual_ = !gradual_;
        rampEdit_.setButtonText(gradual_ ? "Over " + juce::String(EngineViewport::cylinderRampSeconds, 0) + " s"
                                         : juce::String("Next cycle"));
    };
    resetEdit_.setTooltip("Back to the size this part had when you selected it");
    resetEdit_.onClick = [this] {
        if (!edit_) return;
        pendingFirst_ = originFirst_;
        pendingSecond_ = originSecond_;
        setEditStatus({});
        updateEditButtons();
        sendEdit();
    };
}

void PartInspector::timerCallback() { sendEdit(); }

void PartInspector::sendEdit() {
    stopTimer();
    if (!edit_ || !onApplyEdit) return;
    if (std::abs(pendingFirst_ - edit_->first) < 1.0e-6
        && std::abs(pendingSecond_ - edit_->second) < 1.0e-6)
        return;
    onApplyEdit(pendingFirst_, pendingSecond_, gradual_);
}

void PartInspector::setEdit(std::optional<Edit> edit) {
    const auto keep = edit && edit_ && edit->key == edit_->key && edit->first == edit_->first
        && edit->second == edit_->second;
    const auto resize = edit.has_value() != edit_.has_value()
        || (edit && edit_ && (edit->firstEditable != edit_->firstEditable || edit->note != edit_->note));
    // A live edit comes back as the same component with its new size: the
    // origin stays, so Reset still knows where the editing started.
    const auto sameComponent = edit && edit_ && edit->key == edit_->key;
    if (!keep && edit) {
        pendingFirst_ = edit->first;
        pendingSecond_ = edit->second;
        if (!sameComponent) {
            originFirst_ = edit->first;
            originSecond_ = edit->second;
        }
    }
    if (!keep) {
        editStatus_.clear();
        stopTimer();
    }
    edit_ = std::move(edit);
    for (auto* button : { &firstDown_, &firstUp_ })
        button->setVisible(edit_.has_value() && edit_->firstEditable);
    for (auto* button : { &secondDown_, &secondUp_, &resetEdit_ }) button->setVisible(edit_.has_value());
    rampEdit_.setVisible(edit_.has_value() && edit_->offersRamp);
    if (edit_) {
        const auto tip = [](const juce::String& label, double sign, double step, double fine) {
            const auto mm = [](double value) {
                return juce::String(value, std::abs(value - std::round(value)) > 1.0e-6 ? 1 : 0) + " mm";
            };
            return label + (sign > 0.0 ? " +" : utf8(" \xe2\x88\x92")) + mm(step) + " (Shift: " + mm(fine) + ")";
        };
        firstDown_.setTooltip(tip(edit_->firstLabel, -1.0, edit_->firstStep, edit_->firstFineStep));
        firstUp_.setTooltip(tip(edit_->firstLabel, 1.0, edit_->firstStep, edit_->firstFineStep));
        secondDown_.setTooltip(tip(edit_->secondLabel, -1.0, edit_->secondStep, edit_->secondFineStep));
        secondUp_.setTooltip(tip(edit_->secondLabel, 1.0, edit_->secondStep, edit_->secondFineStep));
    }
    updateEditButtons();
    if (resize) setSize(getWidth(), idealHeight());
    resized();
}

void PartInspector::setEditStatus(juce::String status) {
    if (status == editStatus_) return;
    editStatus_ = std::move(status);
    setSize(getWidth(), idealHeight());
    resized();
    repaint();
}

void PartInspector::stepEdit(bool length, double direction) {
    if (!edit_) return;
    const auto fine = juce::ModifierKeys::currentModifiers.isShiftDown();
    auto& value = length ? pendingFirst_ : pendingSecond_;
    const auto step = length ? (fine ? edit_->firstFineStep : edit_->firstStep)
                             : (fine ? edit_->secondFineStep : edit_->secondStep);
    // Snap to the step, so a 452 mm pipe goes to 460 then 470.
    value = direction > 0.0 ? std::floor(value / step + 1.0e-6) * step + step
                            : std::ceil(value / step - 1.0e-6) * step - step;
    value = std::clamp(value, edit_->minimum, length ? edit_->firstMaximum : edit_->secondMaximum);
    setEditStatus({});
    updateEditButtons();
    startTimer(editSettleMs);
}

void PartInspector::updateEditButtons() {
    const auto changed = edit_ && (std::abs(pendingFirst_ - originFirst_) > 1.0e-6
                                   || std::abs(pendingSecond_ - originSecond_) > 1.0e-6);
    resetEdit_.setEnabled(changed);
    repaint();
}

int PartInspector::editHeight() const {
    if (!edit_) return 0;
    const auto width = static_cast<float>((getWidth() > 0 ? getWidth() : 280) - 24);
    auto height = inspectorEditHeader + inspectorEditRow * (edit_->firstEditable ? 2 : 1) + inspectorEditButtons;
    for (const auto* text : { &edit_->note, &editStatus_ })
        if (text->isNotEmpty())
            height += 4 + static_cast<int>(std::ceil(noteLayout(*text, colours::faint, width).getHeight()));
    return height;
}

void PartInspector::resized() {
    if (!edit_) return;
    auto area = getLocalBounds().reduced(12, 8);
    area = area.removeFromBottom(editHeight());
    area.removeFromTop(inspectorEditHeader);
    const auto place = [&area](ActionButton& down, ActionButton& up) {
        auto row = area.removeFromTop(inspectorEditRow).reduced(0, 2);
        up.setBounds(row.removeFromRight(inspectorStepperWidth));
        row.removeFromRight(inspectorValueWidth);
        down.setBounds(row.removeFromRight(inspectorStepperWidth));
    };
    if (edit_->firstEditable) place(firstDown_, firstUp_);
    place(secondDown_, secondUp_);
    auto buttons = area.removeFromTop(inspectorEditButtons).withTrimmedTop(4);
    resetEdit_.setBounds(buttons.removeFromRight(64));
    buttons.removeFromRight(6);
    rampEdit_.setBounds(buttons.removeFromRight(92));
}

void PartInspector::paintEdit(juce::Graphics& g, juce::Rectangle<int> area) const {
    auto header = area.removeFromTop(inspectorEditHeader);
    g.setColour(colours::line);
    g.drawHorizontalLine(header.getY() + 6, static_cast<float>(header.getX()), static_cast<float>(header.getRight()));
    g.setColour(colours::muted);
    g.setFont(uiFont(10.5F, true).withExtraKerningFactor(0.06F));
    g.drawText(utf8("RESIZE"), header.withTrimmedTop(8), juce::Justification::centredLeft, false);
    const auto row = [&](const juce::String& label, double value, double original) {
        auto line = area.removeFromTop(inspectorEditRow);
        g.setColour(colours::muted);
        g.setFont(uiFont(12.0F));
        g.drawText(label, line, juce::Justification::centredLeft, true);
        line.removeFromRight(inspectorStepperWidth);
        const auto valueArea = line.removeFromRight(inspectorValueWidth);
        g.setColour(std::abs(value - original) > 1.0e-6 ? colours::accentLight : colours::text);
        g.setFont(monoFont(11.5F, false));
        const auto decimals = std::abs(value - std::round(value)) > 1.0e-6 ? 1 : 0;
        g.drawText(juce::String(value, decimals) + " mm", valueArea, juce::Justification::centred, false);
    };
    if (edit_->firstEditable) row(edit_->firstLabel, pendingFirst_, originFirst_);
    row(edit_->secondLabel, pendingSecond_, originSecond_);
    area.removeFromTop(inspectorEditButtons);
    for (const auto* text : { &edit_->note, &editStatus_ }) {
        if (text->isEmpty()) continue;
        area.removeFromTop(4);
        const auto layout = noteLayout(*text, text == &editStatus_ ? colours::critical : colours::faint,
                                       static_cast<float>(area.getWidth()));
        layout.draw(g, area.removeFromTop(static_cast<int>(std::ceil(layout.getHeight()))).toFloat());
    }
}

void PartInspector::setScope(std::optional<Scope> scope) {
    const auto resize = scope.has_value() != scope_.has_value();
    scope_ = std::move(scope);
    if (resize) setSize(getWidth(), idealHeight());
    repaint();
}

void PartInspector::show(juce::String title, juce::String subtitle, std::vector<Row> rows, juce::String note) {
    const auto resize = rows.size() != rows_.size() || note != note_;
    title_ = std::move(title);
    subtitle_ = std::move(subtitle);
    rows_ = std::move(rows);
    note_ = std::move(note);
    if (resize) setSize(getWidth(), idealHeight());
    repaint();
}

juce::TextLayout PartInspector::noteLayout(const juce::String& note, juce::Colour colour, float width) const {
    juce::AttributedString text;
    text.append(note, uiFont(11.0F), colour);
    text.setWordWrap(juce::AttributedString::byWord);
    juce::TextLayout layout;
    layout.createLayout(text, width);
    return layout;
}

int PartInspector::idealHeight() const {
    const auto rows = inspectorHeader + inspectorRowHeight * static_cast<int>(rows_.size()) + 10
        + (scope_ ? inspectorScopeHeight : 0) + editHeight();
    if (note_.isEmpty()) return rows;
    // The viewport gives the inspector 280 px; before that, assume it.
    const auto width = static_cast<float>((getWidth() > 0 ? getWidth() : 280) - 24);
    return rows + 6 + static_cast<int>(std::ceil(noteLayout(note_, colours::faint, width).getHeight()));
}

juce::Rectangle<int> PartInspector::closeArea() const { return { getWidth() - 30, 6, 24, 24 }; }

void PartInspector::mouseUp(const juce::MouseEvent& event) {
    if (closeArea().contains(event.getPosition()) && onClose) onClose();
}

void PartInspector::paint(juce::Graphics& g) {
    auto area = getLocalBounds().toFloat();
    g.setColour(juce::Colour(0xeb0d1211));
    g.fillRoundedRectangle(area, 9.0F);
    g.setColour(colours::lineStrong);
    g.drawRoundedRectangle(area.reduced(0.5F), 9.0F, 1.0F);

    const auto close = closeArea().toFloat().reduced(7.0F);
    g.setColour(colours::muted);
    g.drawLine({ close.getTopLeft(), close.getBottomRight() }, 1.4F);
    g.drawLine({ close.getTopRight(), close.getBottomLeft() }, 1.4F);

    auto content = getLocalBounds().reduced(12, 8);
    g.setColour(colours::text);
    g.setFont(uiFont(13.5F, true));
    g.drawText(title_, content.removeFromTop(18).withTrimmedRight(24), juce::Justification::centredLeft, true);
    g.setColour(colours::faint);
    g.setFont(uiFont(11.5F));
    g.drawText(subtitle_, content.removeFromTop(16).withTrimmedRight(24), juce::Justification::centredLeft, true);
    content.removeFromTop(4);
    for (const auto& row : rows_) {
        auto line = content.removeFromTop(inspectorRowHeight);
        g.setColour(colours::muted);
        g.setFont(uiFont(12.0F));
        g.drawText(row.label, line, juce::Justification::centredLeft, true);
        g.setColour(colours::text);
        g.setFont(monoFont(11.5F, false));
        g.drawText(row.value, line, juce::Justification::centredRight, true);
    }
    if (scope_) paintScope(g, content.removeFromTop(inspectorScopeHeight).toFloat());
    if (edit_) paintEdit(g, content.removeFromBottom(editHeight()));
    if (note_.isNotEmpty()) {
        content.removeFromTop(6);
        noteLayout(note_, colours::faint, static_cast<float>(content.getWidth())).draw(g, content.toFloat());
    }
}

void PartInspector::paintScope(juce::Graphics& g, juce::Rectangle<float> area) const {
    area.removeFromTop(6.0F);
    g.setFont(uiFont(11.0F));
    g.setColour(colours::muted);
    g.drawText(scope_->caption, area.removeFromTop(15.0F), juce::Justification::centredLeft, true);
    const auto ticks = area.removeFromBottom(14.0F);
    auto plot = area.reduced(0.0F, 3.0F);
    g.setColour(juce::Colour(0xff0a0e0d));
    g.fillRoundedRectangle(plot, 4.0F);
    g.setColour(colours::line);
    g.drawRoundedRectangle(plot, 4.0F, 1.0F);
    plot = plot.reduced(4.0F, 6.0F);

    const auto& kpa = scope_->kpa;
    float low = 0.0F, high = 0.0F;
    for (const auto value : kpa)
        if (std::isfinite(value)) {
            low = std::min(low, value);
            high = std::max(high, value);
        }
    // At least 1 kPa either side of ambient, so a quiet duct is not magnified.
    low = std::min(low, -1.0F);
    high = std::max(high, 1.0F);
    const auto yOf = [&](float value) { return plot.getBottom() - (value - low) / (high - low) * plot.getHeight(); };
    const auto bins = static_cast<float>(std::max<std::size_t>(1U, kpa.size()));
    const auto xOf = [&](std::size_t bin) { return plot.getX() + (static_cast<float>(bin) + 0.5F) / bins * plot.getWidth(); };

    // Ambient, and the crank revolutions.
    g.setColour(colours::lineStrong);
    g.drawHorizontalLine(juce::roundToInt(yOf(0.0F)), plot.getX(), plot.getRight());
    const auto revolutions = juce::roundToInt(scope_->cycleDegrees / 360.0);
    for (int r = 1; r < revolutions; ++r) {
        const auto x = plot.getX() + plot.getWidth() * static_cast<float>(r) / static_cast<float>(revolutions);
        g.drawVerticalLine(juce::roundToInt(x), plot.getY(), plot.getBottom());
    }
    juce::Path trace;
    bool open = false;
    for (std::size_t bin = 0; bin < kpa.size(); ++bin) {
        if (!std::isfinite(kpa[bin])) {
            open = false;
            continue;
        }
        const juce::Point<float> point { xOf(bin), yOf(kpa[bin]) };
        if (open) trace.lineTo(point);
        else trace.startNewSubPath(point);
        open = true;
    }
    g.setColour(colours::accentLight);
    g.strokePath(trace, juce::PathStrokeType(1.4F, juce::PathStrokeType::curved));
    if (scope_->latestBin >= 0 && static_cast<std::size_t>(scope_->latestBin) < kpa.size()) {
        const auto x = xOf(static_cast<std::size_t>(scope_->latestBin));
        g.setColour(colours::text.withAlpha(0.5F));
        g.drawVerticalLine(juce::roundToInt(x), plot.getY(), plot.getBottom());
    }
    g.setFont(monoFont(10.0F, false));
    g.setColour(colours::faint);
    const auto kpaText = [](float value) { return (value >= 0.0F ? "+" : "") + juce::String(value, 1) + " kPa"; };
    g.drawText(kpaText(high), plot.withHeight(12.0F), juce::Justification::topRight, false);
    g.drawText(kpaText(low), plot.withTrimmedTop(plot.getHeight() - 12.0F), juce::Justification::bottomRight, false);
    g.setFont(uiFont(10.5F));
    g.drawText(utf8("0°"), ticks, juce::Justification::centredLeft, false);
    g.drawText(utf8("crank angle"), ticks, juce::Justification::centred, false);
    g.drawText(juce::String(juce::roundToInt(scope_->cycleDegrees)) + utf8("°"), ticks,
               juce::Justification::centredRight, false);
}

// ------------------------------------------------------------------ viewport
namespace {
constexpr float fieldOfViewRadians = 32.0F * std::numbers::pi_v<float> / 180.0F;
constexpr std::array<int, 6> frameRateCaps { 30, 60, 120, 144, 240, 0 };
constexpr std::array<double, 4> playbackFactors { 1.0, 1.0 / 50.0, 1.0 / 250.0, 0.0 };

[[nodiscard]] double wallSeconds() noexcept { return juce::Time::getMillisecondCounterHiRes() * 1.0e-3; }

[[nodiscard]] juce::Path settingsIcon() {
    // Three sliders.
    return juce::Drawable::parseSVGPath("M4 6h9M17 6h3M4 12h3M11 12h9M4 18h11M19 18h1"
                                        "M13 6a2 2 0 1 0 4 0a2 2 0 1 0 -4 0"
                                        "M7 12a2 2 0 1 0 4 0a2 2 0 1 0 -4 0"
                                        "M15 18a2 2 0 1 0 4 0a2 2 0 1 0 -4 0");
}

[[nodiscard]] render::Vec3 orbitEye(render::Vec3 target, float yaw, float pitch, float distance) noexcept {
    return target + render::Vec3 { std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw) }
        * distance;
}

[[nodiscard]] juce::String millimetres(double value) { return juce::String(value, value < 100.0 ? 1 : 0) + " mm"; }

[[nodiscard]] juce::String exhaustComponentName(ExhaustComponentType type) {
    switch (type) {
    case ExhaustComponentType::pipe: return "Pipe";
    case ExhaustComponentType::merge: return "Collector";
    case ExhaustComponentType::splitter: return "Splitter";
    case ExhaustComponentType::resonator: return "Resonator";
    case ExhaustComponentType::muffler: return "Muffler";
    case ExhaustComponentType::catalyst: return "Catalyst";
    case ExhaustComponentType::outlet: return "Outlet";
    case ExhaustComponentType::crossover: return "Crossover";
    }
    return "Component";
}
} // namespace

EngineViewport::EngineViewport(const DashboardModel& model) : model_(model), cutaway_(model) {
    safeThis_ = this;
    // Clicks orbit the camera; the keyboard stays with the main component.
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
    settings_.load();
    layers_.setOptionTooltip(0, utf8("Every layer"));
    layers_.setOptionTooltip(1, utf8("Combustion and valvetrain"));
    layers_.setOptionTooltip(2, utf8("Pistons, rods, crank and valves"));
    layers_.setOptionTooltip(3, utf8("Intake and exhaust flow, metal faded"));
    layers_.onChange = [this](int layer) {
        cutaway_.setLayer(layer);
        pushSettings();
    };
    views_.setOptionTooltip(0, utf8("Look along the crankshaft"));
    views_.setOptionTooltip(1, utf8("Look across the crankshaft"));
    views_.setOptionTooltip(2, utf8("Three-quarter view"));
    views_.setOptionTooltip(3, utf8("The whole exhaust and intake, laid out from the configuration"));
    views_.setSelected(view_);
    views_.onChange = [this](int view) { applyView(view, true); };
    shading_.setOptionTooltip(0, utf8("See-through block and heads"));
    shading_.setOptionTooltip(1, utf8("Opaque block and heads"));
    shading_.onChange = [this](int) { pushSettings(); };
    playback_.setOptionTooltip(0, utf8("The crank turns at the simulated speed"));
    playback_.setOptionTooltip(1, utf8("Fifty times slower than the engine"));
    playback_.setOptionTooltip(2, utf8("250 times slower than the engine"));
    playback_.setOptionTooltip(3, utf8("Hold the crank where it is"));
    playback_.onChange = [this](int) { pushSettings(); };
    settingsButton_.setIcon(settingsIcon());
    settingsButton_.setTooltip(utf8("Renderer, frame rate, VSync, motion blur, anti-aliasing"));
    settingsButton_.onClick = [this] { showSettingsMenu(); };
    cycle_.phaseOf = [this](std::size_t cylinder) {
        const auto angle = context_.getTargetComponent() != nullptr ? displayedAngle_.load()
                                                                    : model_.state.crankAngleDegrees;
        if (scene_ && cylinder < scene_->cylinderCount()) return scene_->cyclePhaseDegrees(cylinder, angle);
        return 0.0;
    };
    cycle_.setInterceptsMouseClicks(false, false);
    legend_.setInterceptsMouseClicks(false, false);
    inspector_.onClose = [this] { setSelectedPart(-1); };
    inspector_.onApplyEdit = [this](double first, double second, bool gradual) { applyEdit(first, second, gradual); };

    addChildComponent(cutaway_);
    addChildComponent(legend_);
    addChildComponent(inspector_);
    for (auto* overlay : std::initializer_list<juce::Component*> { &layers_, &views_, &shading_, &playback_,
                                                                   &settingsButton_, &cycle_ })
        addAndMakeVisible(overlay);

    context_.setRenderer(this);
    context_.setOpenGLVersionRequired(juce::OpenGLContext::openGL3_2);
    context_.setComponentPaintingEnabled(true);
    context_.setContinuousRepainting(false);
    pushSettings();
    applyRenderer();
}

EngineViewport::~EngineViewport() {
    stopPacing();
    context_.detach();
}

void EngineViewport::setWheelModifierCheck(std::function<bool()> check) {
    cutaway_.wheelModifierActive = check;
    wheelModifierActive_ = std::move(check);
}

void EngineViewport::setGasFieldSource(std::function<bool(double, GasFieldSnapshot&)> source) {
    gasFieldSource_ = std::move(source);
}

void EngineViewport::setGasProbeSource(std::function<bool(std::int32_t, std::uint8_t, GasProbeTrace&)> source) {
    gasProbeSource_ = std::move(source);
}

void EngineViewport::setConfigEditor(std::function<bool(const EngineConfig&)> editor) {
    configEditor_ = std::move(editor);
}

void EngineViewport::setCylinderEditor(std::function<bool(const EngineConfig&, double)> editor) {
    cylinderEditor_ = std::move(editor);
}

void EngineViewport::applyEdit(double first, double second, bool gradual) {
    if (!scene_ || selectedPart_ < 0) return;
    const auto role = scene_->identify(static_cast<std::uint16_t>(selectedPart_)).role;
    if (role == render::PartRole::duct) applyExhaustEdit(first, second);
    else if (role == render::PartRole::liner || role == render::PartRole::piston)
        applyCylinderEdit(first, second, gradual);
}

void EngineViewport::applyCylinderEdit(double boreMm, double strokeMm, bool gradual) {
    if (!scene_ || !cylinderEditor_ || selectedPart_ < 0) return;
    auto edited = scene_->config();
    if (const auto error = resizeCylinders(edited, boreMm, strokeMm); !error.empty()) {
        inspector_.setEditStatus(juce::String::fromUTF8(error.c_str()));
        return;
    }
    if (const auto invalid = validateEngineConfig(edited)) {
        inspector_.setEditStatus(juce::String::fromUTF8(invalid->c_str()));
        return;
    }
    reselect_ = Reselect { 0, 0, selectedPart_ };
    if (!cylinderEditor_(edited, gradual ? cylinderRampSeconds : 0.0)) {
        reselect_.reset();
        inspector_.setEditStatus("The engine did not take the edit.");
    }
}

void EngineViewport::applyExhaustEdit(double lengthMm, double diameterMm) {
    if (!scene_ || !configEditor_ || selectedPart_ < 0) return;
    const auto identity = scene_->identify(static_cast<std::uint16_t>(selectedPart_));
    if (identity.role != render::PartRole::duct) return;
    // A copy: applying the edit replaces the scene.
    const auto duct = scene_->ducts()[static_cast<std::size_t>(identity.duct)];
    auto edited = scene_->config();
    if (const auto error = resizeExhaustComponent(edited, duct.pathId, duct.elementId, lengthMm, diameterMm);
        !error.empty()) {
        inspector_.setEditStatus(juce::String::fromUTF8(error.c_str()));
        return;
    }
    if (const auto invalid = validateEngineConfig(edited)) {
        inspector_.setEditStatus(juce::String::fromUTF8(invalid->c_str()));
        return;
    }
    reselect_ = Reselect { duct.pathId, duct.elementId };
    if (!configEditor_(edited)) {
        reselect_.reset();
        inspector_.setEditStatus("The engine did not take the edit.");
    }
}

void EngineViewport::setEngine(const EngineConfig& config) {
    // After an edit made here, the camera stays and the part stays selected.
    const auto reselect = std::exchange(reselect_, std::nullopt);
    try {
        scene_ = std::make_shared<const render::EngineModel3D>(config);
    } catch (const std::exception&) {
        scene_.reset();
    }
    std::vector<juce::String> labels;
    for (const auto& cylinder : config.cylinders) labels.push_back("C" + juce::String(cylinder.id));
    cycle_.setCylinders(std::move(labels));
    {
        const std::lock_guard lock(sharedMutex_);
        shared_.model = scene_;
        shared_.modelRevision = ++modelRevision_;
        shared_.gasField.reset();
        shared_.selectedPart = -1;
        if (scene_ && !reselect) {
            shared_.goal = presetOrbit(view_);
            shared_.smoothingSeconds = 0.0F;
        }
    }
    gasField_ = {};
    gasFieldBinding_.clear();
    selectedPart_ = -1;
    inspector_.setVisible(false);
    pressureScalePa_ = 0.0F;
    resized();
    cutaway_.repaint();
    if (reselect && scene_ && reselect->part >= 0) {
        if (reselect->part < static_cast<int>(scene_->parts().size())) setSelectedPart(reselect->part);
    } else if (reselect && scene_)
        for (const auto& duct : scene_->ducts())
            if (duct.kind == render::DuctKind::exhaustComponent && duct.pathId == reselect->pathId
                && duct.elementId == reselect->elementId) {
                setSelectedPart(static_cast<int>(duct.part));
                break;
            }
}

void EngineViewport::stepLayer(int delta) {
    layers_.setSelected(std::clamp(layers_.selected() + delta, 0, 3), juce::sendNotificationSync);
}

void EngineViewport::pushSettings() {
    const std::lock_guard lock(sharedMutex_);
    shared_.layer = static_cast<SceneLayerMode>(std::clamp(layers_.selected(), 0, 3));
    shared_.xray = shading_.selected() == 0;
    shared_.playbackFactor = playbackFactors[static_cast<std::size_t>(std::clamp(playback_.selected(), 0, 3))];
    shared_.motionBlur = settings_.motionBlur;
    shared_.antiAliasing = settings_.antiAliasing;
    shared_.vsync = settings_.vsync;
}

void EngineViewport::applyRenderer() {
    const auto use3d = settings_.renderer3d && !glFailed_;
    if (use3d) {
        if (context_.getTargetComponent() == nullptr) {
            framesRendered_ = 0;
            initialiseFailed_ = false;
            attachedAt_ = wallSeconds();
            context_.attachTo(*this);
        }
        cutaway_.setVisible(false);
        startPacing();
    } else {
        stopPacing();
        if (context_.getTargetComponent() != nullptr) context_.detach();
        cutaway_.setVisible(true);
    }
    updateOverlayVisibility();
    lastFpsWall_ = 0.0;
    repaint();
}

void EngineViewport::updateOverlayVisibility() {
    const auto use3d = context_.getTargetComponent() != nullptr;
    views_.setVisible(use3d);
    shading_.setVisible(use3d);
    playback_.setVisible(use3d);
    updateLegend();
    if (!use3d) setSelectedPart(-1);
}

void EngineViewport::updateLegend() {
    const auto layer = layers_.selected();
    const auto show = context_.getTargetComponent() != nullptr && pressureScalePa_.load() > 0.0F
        && (layer == static_cast<int>(SceneLayerMode::all) || layer == static_cast<int>(SceneLayerMode::gasFlow));
    legend_.setScale(pressureScalePa_.load());
    if (show != legend_.isVisible()) {
        legend_.setVisible(show);
        repaint(infoArea().getUnion(infoArea().withY(getHeight() - 30)));
    }
}

void EngineViewport::startPacing() {
    const auto cap = settings_.frameRateCap;
    context_.setContinuousRepainting(cap == 0);
    if (cap == 0) {
        stopPacing();
        return;
    }
    pacerPeriodMicroseconds_ = 1'000'000 / cap;
    if (pacer_.joinable()) return;
    {
        const std::lock_guard lock(pacerMutex_);
        pacerStop_ = false;
    }
    pacer_ = std::thread([this] { pacingLoop(); });
}

void EngineViewport::stopPacing() {
    {
        const std::lock_guard lock(pacerMutex_);
        pacerStop_ = true;
    }
    pacerWake_.notify_all();
    if (pacer_.joinable()) pacer_.join();
}

void EngineViewport::pacingLoop() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    auto lastOverlay = next;
    std::unique_lock lock(pacerMutex_);
    while (!pacerStop_) {
        const auto period = std::chrono::microseconds(pacerPeriodMicroseconds_.load());
        next += period;
        // After a stall, restart the cadence instead of firing a burst.
        if (const auto now = clock::now(); next < now - std::chrono::milliseconds(50)) next = now;
        if (pacerWake_.wait_until(lock, next, [this] { return pacerStop_; })) break;
        // Repainting an overlay renders a frame by itself. Doing it on a tick,
        // instead of from the 30 Hz timer, keeps one frame per tick and an
        // even cadence.
        if (clock::now() - lastOverlay >= std::chrono::milliseconds(33) - period / 2) {
            lastOverlay = clock::now();
            juce::MessageManager::callAsync([safe = safeThis_] {
                if (safe != nullptr) safe->cycle_.repaint();
            });
        } else {
            context_.triggerRepaint();
        }
    }
}

EngineViewport::Orbit EngineViewport::presetOrbit(int view) const {
    if (!scene_) return {};
    view = std::clamp(view, 0, 3);
    // Front looks at the crank nose (the flywheel is at +Z); the 3/4 view is
    // from the nose side too, so the exhaust runs away from the camera.
    static constexpr std::array<render::Vec3, 3> directions { render::Vec3 { -0.02F, 0.18F, -1.0F },
                                                              render::Vec3 { 1.0F, 0.18F, 0.02F },
                                                              render::Vec3 { 1.05F, 0.55F, -0.85F } };
    auto direction = view < 3 ? directions[static_cast<std::size_t>(view)] : render::Vec3 {};
    if (view == 3) {
        // The whole exhaust and intake, from the side the exhaust runs on.
        float sum = 0.0F;
        std::size_t count = 0;
        for (const auto& duct : scene_->ducts()) {
            if (duct.kind != render::DuctKind::exhaustComponent) continue;
            for (const auto& point : duct.centreline) sum += point.x;
            count += duct.centreline.size();
        }
        const auto side = count > 0 && sum / static_cast<float>(count) < scene_->bounds().centre().x - 1.0F ? -1.0F : 1.0F;
        direction = { side, 0.5F, 0.3F };
    }
    direction = render::normalise(direction);
    const auto& bounds = view == 3 ? scene_->systemBounds() : scene_->bounds();
    Orbit orbit;
    orbit.target = bounds.centre();
    orbit.pitch = std::asin(direction.y);
    orbit.yaw = std::atan2(direction.x, direction.z);
    orbit.distance = 0.5F * bounds.diagonal() / std::sin(0.5F * fieldOfViewRadians) * (view == 3 ? 0.8F : 0.9F);
    return orbit;
}

EngineViewport::Orbit EngineViewport::goal() const {
    const std::lock_guard lock(sharedMutex_);
    return shared_.goal;
}

void EngineViewport::setGoal(const Orbit& orbit, float smoothingSeconds) {
    const std::lock_guard lock(sharedMutex_);
    shared_.goal = orbit;
    shared_.smoothingSeconds = smoothingSeconds;
}

void EngineViewport::applyView(int view, bool animate) {
    view_ = std::clamp(view, 0, 3);
    if (!scene_) return;
    auto orbit = presetOrbit(view_);
    // Turn the short way round.
    const auto current = goal().yaw;
    constexpr auto twoPi = 2.0F * std::numbers::pi_v<float>;
    orbit.yaw += twoPi * std::round((current - orbit.yaw) / twoPi);
    setGoal(orbit, animate ? 0.22F : 0.0F);
}

void EngineViewport::mouseDown(const juce::MouseEvent& event) {
    dragStart_ = event.position;
    dragged_ = false;
    dragStartOrbit_ = goal();
    panning_ = event.mods.isRightButtonDown() || event.mods.isMiddleButtonDown();
}

void EngineViewport::mouseDrag(const juce::MouseEvent& event) {
    if (!scene_) return;
    const auto delta = event.position - dragStart_;
    if (delta.getDistanceFromOrigin() > 4.0F) dragged_ = true;
    if (!dragged_) return;
    auto orbit = dragStartOrbit_;
    if (panning_) {
        const auto forward = -render::normalise(orbitEye({}, orbit.yaw, orbit.pitch, 1.0F));
        const auto right = render::normalise(render::cross(forward, { 0.0F, 1.0F, 0.0F }));
        const auto up = render::cross(right, forward);
        const auto worldPerPixel = 2.0F * orbit.distance * std::tan(0.5F * fieldOfViewRadians)
            / static_cast<float>(std::max(1, getHeight()));
        orbit.target = orbit.target - right * (delta.x * worldPerPixel) + up * (delta.y * worldPerPixel);
    } else {
        orbit.yaw -= delta.x * 0.008F;
        orbit.pitch = std::clamp(orbit.pitch + delta.y * 0.008F, -1.45F, 1.45F);
    }
    setGoal(orbit, 0.06F);
}

void EngineViewport::mouseUp(const juce::MouseEvent& event) {
    // A click without a drag selects the part under the pointer.
    if (!dragged_ && !panning_ && event.mods.isLeftButtonDown() && context_.getTargetComponent() != nullptr)
        selectPartAt(event.position);
    panning_ = false;
}

void EngineViewport::selectPartAt(juce::Point<float> position) {
    if (!scene_ || getWidth() <= 0 || getHeight() <= 0) return;
    const auto orbit = goal();
    const auto eye = orbitEye(orbit.target, orbit.yaw, orbit.pitch, orbit.distance);
    const auto forward = render::normalise(orbit.target - eye);
    const auto right = render::normalise(render::cross(forward, { 0.0F, 1.0F, 0.0F }));
    const auto up = render::cross(right, forward);
    const auto halfHeight = std::tan(0.5F * fieldOfViewRadians);
    const auto aspect = static_cast<float>(getWidth()) / static_cast<float>(getHeight());
    const auto x = 2.0F * position.x / static_cast<float>(getWidth()) - 1.0F;
    const auto y = 1.0F - 2.0F * position.y / static_cast<float>(getHeight());
    const auto direction = render::normalise(forward + right * (x * halfHeight * aspect) + up * (y * halfHeight));

    render::ScenePoseInput pose;
    {
        const std::lock_guard lock(sharedMutex_);
        pose = shared_.pose;
    }
    pose.crankAngleDegrees = displayedAngle_.load();
    std::vector<render::SceneInstance> instances;
    scene_->pose(pose, instances);
    const auto layer = static_cast<SceneLayerMode>(std::clamp(layers_.selected(), 0, 3));
    const auto xray = shading_.selected() == 0;
    const auto hit = render::pickPart(*scene_, instances, eye, direction, [this, layer, xray](std::uint16_t part) {
        return scenePartPickable(scene_->parts()[part], layer, xray);
    });
    // The oscilloscope probes the duct where the click landed.
    probeStation_ = 0.5F;
    if (hit) {
        const auto point = eye + direction * hit->distance;
        for (const auto& duct : scene_->ducts()) {
            if (duct.part != hit->part || duct.centreline.size() < 2U) continue;
            float walked = 0.0F, at = 0.0F, nearest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < duct.centreline.size(); ++i) {
                if (i > 0) walked += render::length(duct.centreline[i] - duct.centreline[i - 1U]);
                const auto distance = render::length(duct.centreline[i] - point);
                if (distance < nearest) {
                    nearest = distance;
                    at = walked;
                }
            }
            probeStation_ = walked > 0.0F ? at / walked : 0.5F;
        }
    }
    setSelectedPart(hit ? static_cast<int>(hit->part) : -1);
}

void EngineViewport::setSelectedPart(int part) {
    selectedPart_ = part;
    gasProbe_ = {};
    inspector_.setScope(std::nullopt);
    {
        const std::lock_guard lock(sharedMutex_);
        shared_.selectedPart = part;
    }
    updateInspector();
    inspector_.setVisible(part >= 0);
    resized();
}

void EngineViewport::updateInspector() {
    if (!scene_ || selectedPart_ < 0 || selectedPart_ >= static_cast<int>(scene_->parts().size())) return;
    const auto& config = scene_->config();
    const auto& state = model_.state;
    const auto identity = scene_->identify(static_cast<std::uint16_t>(selectedPart_));
    const auto cylinderIndex = identity.cylinder >= 0 ? static_cast<std::size_t>(identity.cylinder) : 0U;
    const auto* cylinder = identity.cylinder >= 0 && cylinderIndex < config.cylinders.size()
        ? &config.cylinders[cylinderIndex] : nullptr;
    const auto* live = identity.cylinder >= 0 && cylinderIndex < state.cylinderStateCount
        ? &state.cylinderStates[cylinderIndex] : nullptr;
    const auto cylinderName = cylinder != nullptr ? "Cylinder " + juce::String(cylinder->id) : juce::String();
    juce::String title;
    juce::String subtitle = cylinderName;
    juce::String note;
    std::optional<PartInspector::Edit> edit;
    std::vector<PartInspector::Row> rows;
    const auto add = [&rows](juce::String label, juce::String value) { rows.push_back({ std::move(label), std::move(value) }); };
    // Bore and stroke are one engine-wide edit, offered on any cylinder.
    const auto cylinderEdit = [this, &config]() -> std::optional<PartInspector::Edit> {
        const auto size = cylinderSize(config);
        if (!cylinderEditor_ || !size) return std::nullopt;
        PartInspector::Edit result;
        result.key = std::uint64_t { 1 } << 63U;
        result.firstLabel = "Bore";
        result.secondLabel = "Stroke";
        result.first = size->boreMm;
        result.second = size->strokeMm;
        result.firstStep = 1.0;
        result.firstFineStep = 0.1;
        result.minimum = 20.0;
        result.firstMaximum = 200.0;
        result.secondMaximum = 200.0;
        result.offersRamp = true;
        result.note = "Every cylinder changes, each crank throw at its gas-exchange TDC.";
        return result;
    };

    switch (identity.role) {
    case render::PartRole::block:
        title = "Cylinder block";
        subtitle = juce::String(config.name);
        add("Cylinders", juce::String(static_cast<int>(config.cylinders.size())));
        add("Displacement", juce::String(engineDisplacementLitres(config), 2) + " L");
        break;
    case render::PartRole::head:
        title = "Cylinder heads";
        subtitle = juce::String(config.name);
        add("Engine speed", juce::String(juce::roundToInt(state.rpm)) + " rpm");
        break;
    case render::PartRole::liner:
        title = "Cylinder liner";
        edit = cylinderEdit();
        if (cylinder != nullptr) {
            add("Bore", millimetres(cylinder->boreMm));
            add("Stroke", millimetres(cylinder->strokeMm));
            add("Compression ratio", juce::String(cylinder->compressionRatio, 1) + utf8(" : 1"));
        }
        break;
    case render::PartRole::crankshaft:
        title = "Crankshaft";
        subtitle = juce::String(config.name);
        add("Engine speed", juce::String(juce::roundToInt(state.rpm)) + " rpm");
        add("Crank angle", juce::String(juce::roundToInt(displayedAngle_.load())) + utf8("°"));
        break;
    case render::PartRole::piston:
        title = "Piston";
        edit = cylinderEdit();
        if (cylinder != nullptr) {
            add("Bore", millimetres(cylinder->boreMm));
            add("Mass", juce::String(juce::roundToInt(cylinder->pistonMassGrams)) + " g");
        }
        if (live != nullptr) {
            add("Cylinder pressure", juce::String(live->pressureEstimateBar, 1) + " bar");
            add("Gas temperature", juce::String(juce::roundToInt(live->gasTemperatureC)) + utf8(" °""C"));
        }
        break;
    case render::PartRole::rod:
        title = "Connecting rod";
        if (cylinder != nullptr) {
            add("Length", millimetres(cylinder->connectingRodMm));
            add("Mass", juce::String(juce::roundToInt(cylinder->connectingRodMassGrams)) + " g");
        }
        break;
    case render::PartRole::intakeValve:
    case render::PartRole::exhaustValve: {
        const auto intake = identity.role == render::PartRole::intakeValve;
        title = intake ? "Intake valve" : "Exhaust valve";
        const auto& cams = scene_->camshaftsOf(cylinderIndex);
        add("Lift", millimetres(intake ? cams.intakeLiftMm : cams.exhaustLiftMm));
        add("Duration", juce::String(juce::roundToInt(intake ? cams.intakeDurationDegrees : cams.exhaustDurationDegrees))
                            + utf8("°"));
        add("Centreline", juce::String(juce::roundToInt(intake ? cams.intakeCenterlineDegrees : cams.exhaustCenterlineDegrees))
                              + utf8("°"));
        if (live != nullptr)
            add("Lift now", millimetres(intake ? live->intakeValveLiftMm : live->exhaustValveLiftMm));
        break;
    }
    case render::PartRole::combustion:
        title = "Combustion";
        if (live != nullptr) {
            add("Burned fraction", juce::String(juce::roundToInt(100.0 * live->burnedFraction)) + " %");
            add("Flame speed", juce::String(live->flameSpeedMps, 1) + " m/s");
            add("Misfiring", live->misfiring ? "yes" : "no");
        }
        break;
    case render::PartRole::intakePorts:
        title = "Intake ports";
        break;
    case render::PartRole::exhaustPorts:
        title = "Exhaust ports";
        if (live != nullptr) add("Exhaust gas", juce::String(juce::roundToInt(live->exhaustTemperatureC)) + utf8(" °""C"));
        break;
    case render::PartRole::afterfire:
        title = "Afterfire";
        subtitle = "Flame at the outlet";
        add("Heat release", juce::String(state.exhaustAfterfireHeatReleaseKw, 2) + " kW");
        break;
    case render::PartRole::turbo: {
        const auto& forced = config.forcedInduction;
        const auto krpm = [](double rpm) { return juce::String(rpm / 1'000.0, 1) + " krpm"; };
        const auto area = [](double mm2) { return juce::String(juce::roundToInt(mm2)) + utf8(" mm²"); };
        title = "Turbocharger";
        subtitle = "On exhaust path " + juce::String(scene_->turbo().pathId);
        add("Shaft speed", krpm(state.forcedInductionShaftSpeedRpm));
        add("Pressure ratio", juce::String(state.boostPressureRatio, 2) + " (target " + juce::String(forced.pressureRatio, 2) + ")");
        add("Turbine outlet", juce::String(state.turbineOutletPressureKpa, 1) + " kPa");
        add("Wastegate", juce::String(juce::roundToInt(100.0 * state.wastegateOpening)) + " % open");
        add("Turbine / compressor", juce::String(state.turbinePowerKw, 1) + " / " + juce::String(state.compressorPowerKw, 1) + " kW");
        add("Design shaft speed", krpm(forced.designShaftSpeedRpm));
        add("Turbine flow area", area(forced.turbineFlowAreaMm2));
        add("Wastegate flow area", area(forced.wastegateFlowAreaMm2));
        add("Blades (compressor / turbine)", forced.compressorBladeCount > 0U && forced.turbineBladeCount > 0U
                ? juce::String(static_cast<int>(forced.compressorBladeCount)) + " / "
                    + juce::String(static_cast<int>(forced.turbineBladeCount))
                : juce::String("not set"));
        if (forced.compressorInducerDiameterMm > 0.0 && forced.turbineExducerDiameterMm > 0.0)
            add("Inducer / exducer", juce::String(juce::roundToInt(forced.compressorInducerDiameterMm)) + " / "
                                         + millimetres(forced.turbineExducerDiameterMm));
        note = "The gas solver has no turbine in its network: the turbine and open wastegate areas narrow every "
               "exhaust outlet, as a restriction in series, and the shaft is driven by the exhaust pressure and "
               "flow. Drawn where a real one sits, after the collector; the charge piping is not drawn.";
        break;
    }
    case render::PartRole::duct: {
        const auto& duct = scene_->ducts()[static_cast<std::size_t>(identity.duct)];
        switch (duct.kind) {
        case render::DuctKind::exhaustComponent: title = exhaustComponentName(duct.componentType); break;
        case render::DuctKind::exhaustFeeder: title = "Junction"; break;
        case render::DuctKind::intakeRunner: title = "Intake runner"; break;
        case render::DuctKind::intakePlenum: title = "Plenum"; break;
        case render::DuctKind::intakeThrottle: title = "Throttle body"; break;
        case render::DuctKind::intakeAirbox: title = "Airbox"; break;
        case render::DuctKind::intakeInletDuct: title = "Air inlet"; break;
        }
        const auto exhaust = duct.kind == render::DuctKind::exhaustComponent || duct.kind == render::DuctKind::exhaustFeeder;
        if (duct.kind == render::DuctKind::exhaustComponent && configEditor_) {
            if (const auto size = exhaustComponentSize(config, duct.pathId, duct.elementId)) {
                edit = PartInspector::Edit {};
                edit->key = (static_cast<std::uint64_t>(duct.pathId) << 32U) | duct.elementId;
                edit->first = size->lengthMm;
                edit->second = size->diameterMm;
                edit->firstEditable = size->lengthEditable;
                if (size->sharedByPrimaries)
                    edit->note = "The configuration gives the primaries of a path one size: all of them change.";
            }
        }
        subtitle = (exhaust ? "Exhaust path " : "Intake path ") + juce::String(duct.pathId);
        if (duct.kind == render::DuctKind::intakeRunner) subtitle = cylinderName;
        if (duct.authoredLengthMm > 1.0F) add("Length", millimetres(duct.authoredLengthMm));
        if (exhaust) {
            for (const auto& path : config.exhaustPaths) {
                if (path.id != duct.pathId) continue;
                const auto network = path.network ? *path.network : makeEditableExhaustNetwork(path);
                for (const auto& component : network.components) {
                    if (component.id != duct.elementId || duct.kind != render::DuctKind::exhaustComponent) continue;
                    add("Diameter", millimetres(component.diameterMm));
                    if (component.volumeLitres > 0.0) add("Volume", juce::String(component.volumeLitres, 2) + " L");
                }
            }
        }
        if (duct.kind == render::DuctKind::intakePlenum || duct.kind == render::DuctKind::intakeAirbox)
            for (const auto& path : config.intakePaths)
                if (path.id == duct.pathId)
                    add("Volume", juce::String(duct.kind == render::DuctKind::intakePlenum ? path.geometry.plenumVolumeLitres
                                                                                         : path.geometry.airboxVolumeLitres, 2)
                                      + " L");
        // The solver's gas in it, at the angle the view shows.
        const auto index = static_cast<std::size_t>(identity.duct);
        if (index < gasFieldBinding_.size() && gasFieldBinding_[index] >= 0) {
            const auto& element = gasField_.elements[static_cast<std::size_t>(gasFieldBinding_[index])];
            const auto ambient = static_cast<float>(gasField_.ambientPressurePa);
            float low = 1.0e9F, high = -1.0e9F, gas = 0.0F, wall = 0.0F;
            for (std::size_t s = 0; s < element.sampleCount; ++s) {
                low = std::min(low, element.pressurePa[s] - ambient);
                high = std::max(high, element.pressurePa[s] - ambient);
                gas += element.gasTemperatureK[s];
                wall = std::max(wall, element.wallTemperatureK[s]);
            }
            if (element.sampleCount > 0) {
                const auto kpa = [](float pa) { return (pa >= 0.0F ? "+" : "") + juce::String(pa / 1'000.0F, 1); };
                add("Pressure vs ambient", element.sampleCount > 1 ? kpa(low) + " to " + kpa(high) + " kPa" : kpa(low) + " kPa");
                add("Gas temperature",
                    juce::String(juce::roundToInt(gas / static_cast<float>(element.sampleCount) - 273.15F)) + utf8(" °""C"));
                if (wall > 0.0F) add("Wall temperature", juce::String(juce::roundToInt(wall - 273.15F)) + utf8(" °""C"));
                add("Solver cells", juce::String(static_cast<int>(element.sampleCount)));
            }
        }
        break;
    }
    }
    inspector_.show(title, subtitle, std::move(rows), note);
    inspector_.setEdit(std::move(edit));
}

void EngineViewport::updateScope() {
    if (!scene_ || !gasProbeSource_ || selectedPart_ < 0) return;
    const auto identity = scene_->identify(static_cast<std::uint16_t>(selectedPart_));
    const auto index = static_cast<std::size_t>(std::max(0, identity.duct));
    if (identity.role != render::PartRole::duct || index >= gasFieldBinding_.size() || gasFieldBinding_[index] < 0) return;
    const auto element = gasFieldBinding_[index];
    const auto count = static_cast<int>(gasField_.elements[static_cast<std::size_t>(element)].sampleCount);
    if (count == 0) return;
    const auto sample = static_cast<std::uint8_t>(std::clamp(static_cast<int>(probeStation_ * static_cast<float>(count)), 0, count - 1));
    if (!gasProbeSource_(element, sample, gasProbe_) || gasProbe_.element != element || gasProbe_.sample != sample) return;
    PartInspector::Scope scope;
    scope.kpa.resize(GasProbeTrace::bins);
    for (std::size_t bin = 0; bin < GasProbeTrace::bins; ++bin) {
        const auto pressure = gasProbe_.pressurePa[bin];
        scope.kpa[bin] = pressure > 0.0F ? (pressure - static_cast<float>(gasProbe_.ambientPressurePa)) / 1'000.0F
                                         : std::numeric_limits<float>::quiet_NaN();
    }
    scope.latestBin = gasProbe_.latestBin;
    scope.cycleDegrees = gasProbe_.cycleDegrees;
    scope.caption = count > 1 ? "Pressure, cell " + juce::String(sample + 1) + " of " + juce::String(count)
                                    + ", at every solver step"
                              : juce::String("Pressure, at every solver step");
    inspector_.setScope(std::move(scope));
}

void EngineViewport::mouseWheelMove(const juce::MouseEvent& event, const juce::MouseWheelDetails& wheel) {
    if (!scene_ || (wheelModifierActive_ && wheelModifierActive_())) {
        juce::Component::mouseWheelMove(event, wheel);
        return;
    }
    auto orbit = goal();
    const auto home = presetOrbit(view_).distance;
    orbit.distance = std::clamp(orbit.distance * std::exp(-wheel.deltaY * 0.9F), 0.15F * home, 8.0F * home);
    setGoal(orbit, 0.08F);
}

void EngineViewport::mouseDoubleClick(const juce::MouseEvent&) { applyView(view_, true); }

void EngineViewport::showSettingsMenu() {
    const auto gpuAvailable = !glFailed_;
    const auto use3d = settings_.renderer3d && gpuAvailable;
    juce::PopupMenu menu;
    menu.addSectionHeader(utf8("Renderer"));
    menu.addItem(1, gpuAvailable ? utf8("3-D (GPU)") : utf8("3-D (GPU) \xe2\x80\x94 unavailable"), gpuAvailable, use3d);
    menu.addItem(2, utf8("2-D cutaway"), true, !use3d);
    menu.addSectionHeader(utf8("Frame rate"));
    for (std::size_t i = 0; i < frameRateCaps.size(); ++i) {
        const auto cap = frameRateCaps[i];
        menu.addItem(10 + static_cast<int>(i), cap == 0 ? utf8("Unlimited") : juce::String(cap) + " fps", use3d,
                     settings_.frameRateCap == cap);
    }
    menu.addSeparator();
    menu.addItem(20, utf8("VSync"), use3d, settings_.vsync);
    menu.addItem(21, utf8("Motion blur"), use3d, settings_.motionBlur);
    menu.addItem(22, utf8("Anti-aliasing (4\xc3\x97 MSAA)"), use3d, settings_.antiAliasing);
    menu.showMenuAsync(juce::PopupMenu::Options {}.withTargetComponent(&settingsButton_),
        [safe = juce::Component::SafePointer<EngineViewport>(this)](int result) {
            if (safe == nullptr || result == 0) return;
            auto& settings = safe->settings_;
            if (result == 1 || result == 2) {
                settings.renderer3d = result == 1;
                safe->applyRenderer();
            } else if (result >= 10 && result < 10 + static_cast<int>(frameRateCaps.size())) {
                settings.frameRateCap = frameRateCaps[static_cast<std::size_t>(result - 10)];
                if (safe->context_.getTargetComponent() != nullptr) safe->startPacing();
            } else if (result == 20) {
                settings.vsync = !settings.vsync;
            } else if (result == 21) {
                settings.motionBlur = !settings.motionBlur;
            } else if (result == 22) {
                settings.antiAliasing = !settings.antiAliasing;
            }
            settings.save();
            safe->pushSettings();
        });
}

void EngineViewport::refresh() {
    const auto now = wallSeconds();
    const auto use3d = context_.getTargetComponent() != nullptr;
    std::shared_ptr<const GasFieldSnapshot> newField;
    if (use3d && gasFieldSource_) {
        // In real time the latest field; slowed down or frozen, the field at
        // the angle on screen, from the most recent cycle.
        const auto realTime = playback_.selected() == 0;
        if (gasFieldSource_(realTime ? -1.0 : displayedAngle_.load(), gasField_)) {
            if (scene_ && gasFieldBinding_.size() != scene_->ducts().size())
                gasFieldBinding_ = render::bindGasField(*scene_, gasField_);
            newField = std::make_shared<const GasFieldSnapshot>(gasField_);
        }
    }
    if (use3d) {
        const auto& state = model_.state;
        const auto& health = model_.health;
        const auto firing = (state.ecuSparkEnabled || model_.config.fuel == FuelType::diesel) && state.ecuFuelEnabled
            && !state.ecuDecelerationFuelCutActive && state.rpm > 60.0;
        const std::lock_guard lock(sharedMutex_);
        if (newField) shared_.gasField = std::move(newField);
        shared_.sampleWall = now;
        shared_.crankAngle = state.crankAngleDegrees;
        shared_.rpm = state.rpm;
        // Simulated seconds per wall second: the time scale, slowed further
        // when the physics cannot keep up.
        shared_.rate = health.paused ? 0.0 : health.timeScale * std::clamp(health.realtimeFactor, 0.0, 1.0);
        shared_.sampleSequence = ++sampleSequence_;
        shared_.pose.rpm = state.rpm;
        shared_.pose.throttle = state.throttle;
        shared_.turboShaftRpm = state.forcedInductionShaftSpeedRpm;
        shared_.pose.combustion.fill(0.0F);
        const auto count = std::min(state.cylinderStateCount, shared_.pose.combustion.size());
        for (std::size_t i = 0; i < count; ++i) {
            const auto& cylinder = state.cylinderStates[i];
            const auto efficiency = std::clamp(cylinder.combustionEfficiency, 0.0, 1.0);
            shared_.pose.combustion[i] = firing && !cylinder.misfiring && efficiency > 0.05
                ? static_cast<float>(0.55 + 0.45 * efficiency) : 0.0F;
        }
    } else {
        cutaway_.repaint();
    }

    // A context that never draws (no OpenGL 3.2 driver) or fails to compile
    // its shaders falls back to the 2-D cutaway.
    if (use3d && (initialiseFailed_.load()
                  || (framesRendered_.load() == 0 && isShowing() && now - attachedAt_ > 5.0))) {
        glFailed_ = true;
        {
            const std::lock_guard lock(sharedMutex_);
            if (glError_.isEmpty()) glError_ = "no OpenGL 3.2 context";
        }
        applyRenderer();
    }

    if (!pacer_.joinable()) cycle_.repaint();
    updateLegend();
    if (inspector_.isVisible()) {
        updateInspector();
        updateScope();
    }

    if (now - lastFpsWall_ >= 1.0) {
        const auto frames = framesRendered_.load();
        measuredFps_ = lastFpsWall_ > 0.0 ? static_cast<double>(frames - lastFrameCount_) / (now - lastFpsWall_) : 0.0;
        lastFrameCount_ = frames;
        lastFpsWall_ = now;
        juce::String text;
        if (context_.getTargetComponent() != nullptr) {
            text = juce::String(juce::roundToInt(measuredFps_))
                + utf8(" fps  \xc2\xb7  drag to orbit, right-drag to pan, wheel to zoom, double-click to reset");
        } else if (glFailed_) {
            const std::lock_guard lock(sharedMutex_);
            text = utf8("3-D view unavailable (") + glError_.upToFirstOccurrenceOf("\n", false, false)
                + utf8("), showing the 2-D cutaway");
        } else {
            text = utf8("2-D cutaway  \xc2\xb7  wheel to zoom, drag to pan, double-click to recentre");
        }
        if (text != infoText_) {
            infoText_ = text;
            repaint(infoArea());
        }
    }
}

juce::Rectangle<int> EngineViewport::infoArea() const {
    const auto left = cycle_.getRight() + 12;
    const auto right = playback_.isVisible() ? playback_.getX() - 12 : getWidth() - 12;
    // Above the legend when it shows.
    const auto bottom = legend_.isVisible() ? legend_.getY() - 6 : getHeight() - 12;
    return { left, bottom - 18, std::max(0, right - left), 18 };
}

void EngineViewport::paint(juce::Graphics& g) {
    if (context_.getTargetComponent() == nullptr) {
        const auto bounds = getLocalBounds().toFloat();
        g.setGradientFill(juce::ColourGradient(juce::Colour(0xff18211f), bounds.getCentreX(),
            bounds.getY() + bounds.getHeight() * 0.4F, colours::background,
            bounds.getCentreX() + bounds.getWidth() * 0.7F, bounds.getY() + bounds.getHeight() * 0.4F, true));
        g.fillRect(bounds);
    }
    g.setColour(colours::faint);
    g.setFont(uiFont(12.5F));
    g.drawText(infoText_, infoArea(), juce::Justification::centredBottom, true);
}

void EngineViewport::resized() {
    cutaway_.setBounds(getLocalBounds());
    layers_.setBounds(12, 12, layers_.idealWidth(), 32);
    auto right = getWidth() - 12;
    settingsButton_.setBounds(right - 36, 12, 36, 32);
    right -= 36 + 6;
    shading_.setBounds(right - shading_.idealWidth(), 12, shading_.idealWidth(), 32);
    right -= shading_.idealWidth() + 6;
    views_.setBounds(right - views_.idealWidth(), 12, views_.idealWidth(), 32);
    playback_.setBounds(getWidth() - 12 - playback_.idealWidth(), getHeight() - 44, playback_.idealWidth(), 32);
    const auto cycleHeight = cycle_.idealHeight();
    cycle_.setBounds(12, getHeight() - 12 - cycleHeight, 250, cycleHeight);
    legend_.setBounds((getWidth() - 300) / 2, getHeight() - 12 - 58, 300, 58);
    inspector_.setBounds(getWidth() - 12 - 280, 12 + 32 + 10, 280, inspector_.idealHeight());
}

// ------------------------------------------------------------ OpenGL thread
void EngineViewport::newOpenGLContextCreated() {
    renderer_ = std::make_unique<EngineSceneRenderer>();
    juce::String error;
    if (!renderer_->initialise(error)) {
        renderer_.reset();
        {
            const std::lock_guard lock(sharedMutex_);
            glError_ = error.isEmpty() ? juce::String("shader compilation failed") : error;
        }
        initialiseFailed_ = true;
        return;
    }
    renderedModelRevision_ = 0;
    observedSequence_ = 0;
    appliedSwapInterval_ = -1;
    orbitValid_ = false;
    lastFrameWall_ = 0.0;
    clock_.reset();
}

void EngineViewport::renderOpenGL() {
    using namespace juce::gl;
    if (!renderer_) return;
    Shared shared;
    {
        const std::lock_guard lock(sharedMutex_);
        shared = shared_;
    }
    if (shared.modelRevision != renderedModelRevision_) {
        renderer_->setModel(shared.model);
        renderedModelRevision_ = shared.modelRevision;
        orbitValid_ = false;
        observedSequence_ = 0;
        clock_.reset();
        gasFieldView_.clear();
        renderedGasField_ = 0;
    }
    if (shared.gasField && shared.gasField->sequence != renderedGasField_ && renderer_->model() != nullptr) {
        gasFieldView_.update(*renderer_->model(), *shared.gasField);
        renderedGasField_ = shared.gasField->sequence;
        pressureScalePa_.store(gasFieldView_.pressureScalePa());
    }
    if (shared.sampleSequence != observedSequence_) {
        clock_.observe(shared.sampleWall, shared.crankAngle, shared.rpm, shared.rate);
        observedSequence_ = shared.sampleSequence;
    }
    clock_.setPlaybackFactor(shared.playbackFactor);
    const auto now = wallSeconds();
    const auto angle = clock_.advance(now);
    const auto dt = lastFrameWall_ > 0.0 ? std::clamp(now - lastFrameWall_, 0.0, 0.25) : 0.0;
    lastFrameWall_ = now;
    if (dt > 0.0) frameInterval_ += (std::clamp(dt, 1.0e-3, 0.1) - frameInterval_) * 0.1;

    const auto swapInterval = shared.vsync ? 1 : 0;
    if (swapInterval != appliedSwapInterval_) {
        (void)context_.setSwapInterval(swapInterval);
        appliedSwapInterval_ = swapInterval;
    }

    if (!orbitValid_ || shared.smoothingSeconds <= 0.0F) {
        orbit_ = shared.goal;
        orbitValid_ = true;
    } else {
        const auto k = static_cast<float>(1.0 - std::exp(-dt / std::max(1.0e-3, static_cast<double>(shared.smoothingSeconds))));
        orbit_.target = render::lerp(orbit_.target, shared.goal.target, k);
        orbit_.yaw += (shared.goal.yaw - orbit_.yaw) * k;
        orbit_.pitch += (shared.goal.pitch - orbit_.pitch) * k;
        orbit_.distance *= std::exp((std::log(shared.goal.distance) - std::log(orbit_.distance)) * k);
    }

    std::array<GLint, 4> viewport {};
    glGetIntegerv(GL_VIEWPORT, viewport.data());
    const auto width = viewport[2];
    const auto height = viewport[3];
    const auto* scene = renderer_->model();
    if (scene == nullptr || width <= 0 || height <= 0) {
        glClearColor(0.043F, 0.059F, 0.055F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        return;
    }

    const auto eye = orbitEye(orbit_.target, orbit_.yaw, orbit_.pitch, orbit_.distance);
    const auto diagonal = scene->systemBounds().diagonal();
    frame_.eye = eye;
    frame_.view = render::Mat4::lookAt(eye, orbit_.target, { 0.0F, 1.0F, 0.0F });
    frame_.projection = render::Mat4::perspective(fieldOfViewRadians,
        static_cast<float>(width) / static_cast<float>(height),
        std::max(0.5F, 0.01F * orbit_.distance), orbit_.distance + 4.0F * diagonal);
    frame_.layer = shared.layer;
    frame_.xray = shared.xray;
    frame_.antiAliasing = shared.antiAliasing;
    frame_.pose = shared.pose;
    frame_.pose.afterfire = gasFieldView_.afterfire();
    // The turbo shaft turns at its simulated speed, slowed down with the crank.
    const auto shaftDegreesPerSecond
        = 6.0 * shared.turboShaftRpm * (shared.playbackFactor >= 0.999 ? shared.rate : shared.playbackFactor);
    turboShaftDegrees_ = std::fmod(turboShaftDegrees_ + shaftDegreesPerSecond * dt, 360.0);
    frame_.pose.turboShaftDegrees = turboShaftDegrees_;
    frame_.gasField = gasFieldView_.valid() ? &gasFieldView_ : nullptr;
    frame_.selectedPart = shared.selectedPart;

    // Motion blur: the shutter stays open for half a frame, and the crank is
    // drawn every 12 degrees it sweeps in that time (12 sub-frames at most).
    // Every 6 degrees, up to 24, took 2.5 % of the physics capacity on a
    // GTX 1660 Super (docs/journal.md).
    const auto sweep = shared.motionBlur ? clock_.displayDegreesPerSecond() * 0.5 * frameInterval_ : 0.0;
    const auto subframes = std::clamp(static_cast<int>(std::ceil(sweep / 12.0)), 1, 12);
    frame_.angles.resize(static_cast<std::size_t>(subframes));
    for (int k = 0; k < subframes; ++k)
        frame_.angles[static_cast<std::size_t>(k)] = angle - sweep * static_cast<double>(subframes - 1 - k) / subframes;
    frame_.turboShaftSweepDegrees = shared.motionBlur ? shaftDegreesPerSecond * 0.5 * frameInterval_ : 0.0;

    renderer_->render(frame_, width, height);
    displayedAngle_.store(angle);
    framesRendered_.fetch_add(1);
}

void EngineViewport::openGLContextClosing() {
    if (renderer_) renderer_->release();
    renderer_.reset();
}

} // namespace enginelab::ui
