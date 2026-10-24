#include <enginelab/app/EngineViewport.hpp>
#include <enginelab/exhaust/ExhaustComponentResize.hpp>
#include <enginelab/foundation/CylinderResize.hpp>
#include <enginelab/foundation/IntakeResize.hpp>
#include <enginelab/foundation/TurboSizing.hpp>
#include <enginelab/foundation/TwoStrokePorts.hpp>
#include <enginelab/render/ListenerFrame.hpp>
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
                    + model_.units.text(units::Quantity::pressure, liveCylinder->pressureEstimateBar * 100.0),
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
        const auto valves = config.cycle != EngineCycle::twoStroke;
        const auto fallbackIntakeLift = !valves ? 0.0 : profiledValveLiftMm(phase, 360.0 + config.camshafts.intakeCenterlineDegrees,
            config.camshafts.intakeDurationDegrees, config.camshafts.intakeLiftMm, config.camshafts.intakeLiftProfile);
        const auto fallbackExhaustLift = !valves ? 0.0 : profiledValveLiftMm(phase, 360.0 - config.camshafts.exhaustCenterlineDegrees,
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
                + model_.units.text(units::Quantity::pressure, cylinderState.pressureEstimateBar * 100.0),
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
    fineExhaustWaves = flag("fineExhaustWaves", fineExhaustWaves);
    // Saved under a new key since it is on by default: every view.json
    // written while it was off by default holds "cameraMicrophone": false
    // without anyone having chosen it.
    cameraMicrophone = flag("microphoneOnCamera", cameraMicrophone);
    const auto waves = object->getProperty("pressureWaves").toString();
    if (waves == "strength") pressureWaves = PressureWaves::strength;
    else if (waves == "live") pressureWaves = PressureWaves::live;
    else pressureWaves = PressureWaves::hidden;
}

void ViewSettings::save() const {
    auto object = std::make_unique<juce::DynamicObject>();
    object->setProperty("renderer", renderer3d ? "3d" : "2d");
    object->setProperty("frameRateCap", frameRateCap);
    object->setProperty("vsync", vsync);
    object->setProperty("motionBlur", motionBlur);
    object->setProperty("antiAliasing", antiAliasing);
    object->setProperty("fineExhaustWaves", fineExhaustWaves);
    object->setProperty("microphoneOnCamera", cameraMicrophone);
    object->setProperty("pressureWaves", pressureWaves == PressureWaves::strength ? "strength"
                                         : pressureWaves == PressureWaves::live   ? "live"
                                                                                  : "hidden");
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

void CyclePanel::setCylinders(std::vector<juce::String> labels, double cycleDegrees) {
    labels_ = std::move(labels);
    cycleDegrees_ = cycleDegrees;
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
        // A two-stroke has two strokes: power (with exhaust and transfer), then
        // compression.
        const auto twoStroke = cycleDegrees_ < 719.0;
        const auto phase = std::clamp(phaseOf(i), 0.0, cycleDegrees_ - 0.001);
        const auto halfStroke = static_cast<std::size_t>(phase / 180.0);
        const auto stroke = twoStroke ? (halfStroke == 0 ? 0U : 3U) : halfStroke;
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
        const auto bands = twoStroke ? std::size_t { 2 } : std::size_t { 4 };
        const auto bandWidth = 1.0F / static_cast<float>(bands);
        for (std::size_t band = 0; band < bands; ++band) {
            g.setColour(strokeBands[twoStroke && band == 1 ? 3 : band]);
            g.fillRect(track.withX(track.getX() + track.getWidth() * static_cast<float>(band) * bandWidth)
                           .withWidth(track.getWidth() * bandWidth));
        }
        g.restoreState();
        g.setColour(juce::Colours::white);
        g.fillRect(juce::Rectangle<float>(track.getX() + track.getWidth() * static_cast<float>(phase / cycleDegrees_) - 1.0F,
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

void WaveLegend::setStrength(bool strength) {
    if (strength == strength_) return;
    strength_ = strength;
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
    g.drawText(strength_ ? utf8("PULSATION STRENGTH") : utf8("PRESSURE WAVES"), header,
               juce::Justification::centredLeft, false);
    g.setFont(monoFont(10.5F, false));
    g.drawText(strength_ ? legendFigure(scalePa_) + " kPa rms" : utf8("±") + legendFigure(scalePa_) + " kPa",
               header, juce::Justification::centredRight, false);

    // The bar of the prototype: violet, grey at the mean, orange, pale yellow.
    // The strength bar starts at the grey: a root mean square is never
    // negative.
    const auto bar = area.removeFromTop(14.0F).withSizeKeepingCentre(area.getWidth(), 8.0F);
    juce::ColourGradient gradient(juce::Colour(strength_ ? 0xff4a5451 : 0xff9a7bff), bar.getX(), 0.0F,
                                  juce::Colour(0xffffe6b0), bar.getRight(), 0.0F, false);
    if (!strength_) gradient.addColour(0.5, juce::Colour(0xff4a5451));
    gradient.addColour(strength_ ? 0.7 : 0.85, juce::Colour(0xffff6a2b));
    g.setGradientFill(gradient);
    g.fillRoundedRectangle(bar, 4.0F);

    g.setFont(uiFont(11.0F));
    g.setColour(colours::muted);
    const auto ticks = area.removeFromTop(14.0F);
    if (strength_) {
        g.drawText(utf8("still"), ticks, juce::Justification::centredLeft, false);
        g.drawText(utf8("strong pulses"), ticks, juce::Justification::centredRight, false);
        return;
    }
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

PartInspector::Field& PartInspector::Edit::length(juce::String label, double millimetres, double step,
                                                  double fineStep, double minimum, double maximum) {
    auto& field = fields.emplace_back();
    field.label = std::move(label);
    field.unit = " mm";
    field.quantity = units::Quantity::length;
    field.value = millimetres;
    field.step = step;
    field.fineStep = fineStep;
    field.minimum = minimum;
    field.maximum = maximum;
    return field;
}

PartInspector::Field& PartInspector::Edit::value(juce::String label, double value, juce::String unit, int decimals,
                                                 double step, double fineStep, double minimum, double maximum) {
    auto& field = fields.emplace_back();
    field.label = std::move(label);
    field.unit = std::move(unit);
    field.decimals = decimals;
    field.value = value;
    field.step = step;
    field.fineStep = fineStep;
    field.minimum = minimum;
    field.maximum = maximum;
    return field;
}

PartInspector::PartInspector() {
    for (auto* button : { &resetEdit_, &rampEdit_, &choiceEdit_ }) addChildComponent(button);
    choiceEdit_.onClick = [this] {
        if (!edit_ || !edit_->choice || edit_->choice->options.empty()) return;
        auto& choice = *edit_->choice;
        choice.index = (choice.index + 1) % static_cast<int>(choice.options.size());
        choiceEdit_.setButtonText(choice.options[static_cast<std::size_t>(choice.index)]);
        if (onChoice) onChoice(choice.index);
    };
    rampEdit_.setTooltip("When the engine takes the change: at the next cycle, or gradually over "
                         + juce::String(EngineViewport::cylinderRampSeconds, 0) + " s");
    const auto showRamp = [this] {
        rampEdit_.setButtonText(gradual_ ? "Over " + juce::String(EngineViewport::cylinderRampSeconds, 0) + " s"
                                         : juce::String("Next cycle"));
    };
    rampEdit_.onClick = [this, showRamp] {
        gradual_ = !gradual_;
        showRamp();
    };
    showRamp();
    valueEditor_.setJustification(juce::Justification::centred);
    valueEditor_.setFont(monoFont(11.5F, false));
    valueEditor_.setIndents(4, 3);
    valueEditor_.setInputRestrictions(9, "-0123456789.,");
    valueEditor_.setColour(juce::TextEditor::backgroundColourId, colours::panelRaised);
    valueEditor_.setColour(juce::TextEditor::textColourId, colours::text);
    valueEditor_.setColour(juce::TextEditor::outlineColourId, colours::accent);
    valueEditor_.setColour(juce::TextEditor::focusedOutlineColourId, colours::accent);
    valueEditor_.onReturnKey = [this] { endTyping(true); };
    valueEditor_.onEscapeKey = [this] { endTyping(false); };
    valueEditor_.onFocusLost = [this] { endTyping(false); };
    addChildComponent(valueEditor_);
    resetEdit_.setTooltip("Back to the values this part had when you selected it");
    resetEdit_.onClick = [this] {
        if (!edit_) return;
        pending_ = origin_;
        setEditStatus({});
        updateEditButtons();
        sendEdit();
    };
}

void PartInspector::timerCallback() { sendEdit(); }

void PartInspector::sendEdit() {
    stopTimer();
    if (!edit_ || !onApplyEdit || pending_.size() != edit_->fields.size()) return;
    auto changed = false;
    for (std::size_t index = 0; index < pending_.size(); ++index)
        changed = changed || std::abs(pending_[index] - edit_->fields[index].value) >= 1.0e-6;
    if (changed) onApplyEdit(pending_, gradual_);
}

void PartInspector::setEdit(std::optional<Edit> edit) {
    const auto sameValues = [](const Edit& left, const Edit& right) {
        if (left.fields.size() != right.fields.size()) return false;
        for (std::size_t index = 0; index < left.fields.size(); ++index)
            if (left.fields[index].value != right.fields[index].value) return false;
        return true;
    };
    const auto keep = edit && edit_ && edit->key == edit_->key && sameValues(*edit, *edit_);
    const auto layoutOf = [](const std::optional<Edit>& candidate) {
        juce::String layout;
        if (!candidate) return layout;
        for (const auto& field : candidate->fields) layout << (field.editable ? "e" : "r");
        return layout << candidate->note;
    };
    const auto resize = layoutOf(edit) != layoutOf(edit_);
    // A live edit comes back as the same part with its new values: the
    // origin stays, so Reset still knows where the editing started.
    const auto samePart = edit && edit_ && edit->key == edit_->key && edit->fields.size() == edit_->fields.size();
    if (!keep && edit) {
        pending_.clear();
        for (const auto& field : edit->fields) pending_.push_back(field.value);
        if (!samePart) origin_ = pending_;
    }
    if (!keep) {
        editStatus_.clear();
        stopTimer();
        if (typing_ >= 0) {
            typing_ = -1;
            valueEditor_.setVisible(false);
        }
    }
    edit_ = std::move(edit);
    const auto fieldCount = edit_ ? edit_->fields.size() : 0U;
    while (downButtons_.size() < fieldCount) {
        const auto index = downButtons_.size();
        auto& down = downButtons_.emplace_back(std::make_unique<ActionButton>("-", ActionButton::Style::compact));
        auto& up = upButtons_.emplace_back(std::make_unique<ActionButton>("+", ActionButton::Style::compact));
        down->onClick = [this, index] { stepEdit(index, -1.0); };
        up->onClick = [this, index] { stepEdit(index, 1.0); };
        addChildComponent(*down);
        addChildComponent(*up);
    }
    for (std::size_t index = 0; index < downButtons_.size(); ++index) {
        const auto shown = index < fieldCount && edit_->fields[index].editable;
        downButtons_[index]->setVisible(shown);
        upButtons_[index]->setVisible(shown);
        if (!shown) continue;
        const auto& field = edit_->fields[index];
        const auto tip = [&](double sign) {
            return field.label + (sign > 0.0 ? " +" : utf8(" \xe2\x88\x92")) + formatValue(index, field.step, true, true)
                + " (Shift: " + formatValue(index, field.fineStep, true, true) + ")";
        };
        downButtons_[index]->setTooltip(tip(-1.0));
        upButtons_[index]->setTooltip(tip(1.0));
    }
    resetEdit_.setVisible(edit_.has_value());
    rampEdit_.setVisible(edit_.has_value() && edit_->offersRamp);
    choiceEdit_.setVisible(edit_.has_value() && edit_->choice.has_value() && !edit_->choice->options.empty());
    if (edit_ && edit_->choice && !edit_->choice->options.empty()) {
        auto& choice = *edit_->choice;
        choice.index = std::clamp(choice.index, 0, static_cast<int>(choice.options.size()) - 1);
        choiceEdit_.setButtonText(choice.options[static_cast<std::size_t>(choice.index)]);
        choiceEdit_.setTooltip(choice.tooltip);
    }
    updateEditButtons();
    if (resize) setSize(getWidth(), idealHeight());
    resized();
}

juce::String PartInspector::formatValue(std::size_t index, double value, bool withUnit, bool difference) const {
    if (!edit_ || index >= edit_->fields.size()) return {};
    const auto& field = edit_->fields[index];
    // A field with a quantity in a unit other than its own: the viewer's
    // unit and its precision.
    if (field.quantity && units_ != nullptr
        && &units_->preferences.of(*field.quantity) != units::unitsOf(*field.quantity).data()) {
        const auto& unit = units_->preferences.of(*field.quantity);
        const auto shown = value * unit.scale + (difference ? 0.0 : unit.offset);
        return juce::String(shown, unit.decimals)
            + (withUnit ? " " + units_->symbol(*field.quantity) : juce::String());
    }
    const auto decimals = field.decimals >= 0 ? field.decimals
                                              : (std::abs(value - std::round(value)) > 1.0e-6 ? 1 : 0);
    return juce::String(value, decimals) + (withUnit ? field.unit : juce::String());
}

void PartInspector::setEditStatus(juce::String status) {
    if (status == editStatus_) return;
    editStatus_ = std::move(status);
    setSize(getWidth(), idealHeight());
    resized();
    repaint();
}

void PartInspector::stepEdit(std::size_t index, double direction) {
    if (!edit_ || index >= edit_->fields.size() || index >= pending_.size()) return;
    const auto& field = edit_->fields[index];
    const auto fine = juce::ModifierKeys::currentModifiers.isShiftDown();
    auto& value = pending_[index];
    const auto step = fine ? field.fineStep : field.step;
    // Snap to the step, so a 452 mm pipe goes to 460 then 470.
    value = direction > 0.0 ? std::floor(value / step + 1.0e-6) * step + step
                            : std::ceil(value / step - 1.0e-6) * step - step;
    value = std::clamp(value, field.minimum, field.maximum);
    setEditStatus({});
    updateEditButtons();
    startTimer(editSettleMs);
}

void PartInspector::updateEditButtons() {
    auto changed = false;
    for (std::size_t index = 0; index < pending_.size() && index < origin_.size(); ++index)
        changed = changed || std::abs(pending_[index] - origin_[index]) > 1.0e-6;
    resetEdit_.setEnabled(edit_.has_value() && changed);
    repaint();
}

int PartInspector::editHeight() const {
    if (!edit_) return 0;
    const auto width = static_cast<float>((getWidth() > 0 ? getWidth() : 280) - 24);
    auto height = inspectorEditHeader + inspectorEditRow * static_cast<int>(edit_->fields.size())
        + inspectorEditButtons;
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
    for (std::size_t index = 0; index < edit_->fields.size(); ++index) {
        auto row = area.removeFromTop(inspectorEditRow).reduced(0, 2);
        if (index >= downButtons_.size()) continue;
        upButtons_[index]->setBounds(row.removeFromRight(inspectorStepperWidth));
        row.removeFromRight(inspectorValueWidth);
        downButtons_[index]->setBounds(row.removeFromRight(inspectorStepperWidth));
    }
    if (typing_ >= 0) valueEditor_.setBounds(valueArea(static_cast<std::size_t>(typing_)));
    auto buttons = area.removeFromTop(inspectorEditButtons).withTrimmedTop(4);
    resetEdit_.setBounds(buttons.removeFromRight(64));
    buttons.removeFromRight(6);
    rampEdit_.setBounds(buttons.removeFromRight(92));
    choiceEdit_.setBounds(rampEdit_.getBounds());
}

void PartInspector::paintEdit(juce::Graphics& g, juce::Rectangle<int> area) const {
    auto header = area.removeFromTop(inspectorEditHeader);
    g.setColour(colours::line);
    g.drawHorizontalLine(header.getY() + 6, static_cast<float>(header.getX()), static_cast<float>(header.getRight()));
    g.setFont(uiFont(10.5F, true).withExtraKerningFactor(0.06F));
    g.setColour(colours::muted);
    g.drawText(edit_->heading, header.withTrimmedTop(8), juce::Justification::centredLeft, false);
    // Whether the engine keeps running through the edit, or restarts.
    g.setColour(edit_->restarts ? colours::warn : colours::ok);
    g.drawText(edit_->restarts ? "RESTARTS" : "LIVE", header.withTrimmedTop(8), juce::Justification::centredRight,
               false);
    for (std::size_t index = 0; index < edit_->fields.size(); ++index) {
        const auto& field = edit_->fields[index];
        auto line = area.removeFromTop(inspectorEditRow);
        g.setColour(colours::muted);
        g.setFont(uiFont(12.0F));
        g.drawText(field.label, line, juce::Justification::centredLeft, true);
        line.removeFromRight(inspectorStepperWidth);
        const auto valueArea = line.removeFromRight(inspectorValueWidth);
        const auto value = index < pending_.size() ? pending_[index] : field.value;
        const auto original = index < origin_.size() ? origin_[index] : field.value;
        // A faint box: the value can be clicked and typed.
        if (field.editable) {
            g.setColour(colours::line);
            g.drawRoundedRectangle(valueArea.reduced(2, 3).toFloat(), 4.0F, 1.0F);
        }
        g.setColour(std::abs(value - original) > 1.0e-6 ? colours::accentLight
                                                         : (field.editable ? colours::text : colours::muted));
        g.setFont(monoFont(11.5F, false));
        g.drawText(formatValue(index, value), valueArea, juce::Justification::centred, false);
    }
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
    if (!edit_ || event.mouseWasDraggedSinceMouseDown()) return;
    for (std::size_t index = 0; index < edit_->fields.size(); ++index) {
        if (edit_->fields[index].editable && valueArea(index).contains(event.getPosition())) {
            beginTyping(index);
            return;
        }
    }
}

void PartInspector::mouseMove(const juce::MouseEvent& event) {
    auto overValue = false;
    if (edit_)
        for (std::size_t index = 0; index < edit_->fields.size(); ++index)
            overValue = overValue || (edit_->fields[index].editable && valueArea(index).contains(event.getPosition()));
    setMouseCursor(overValue ? juce::MouseCursor::IBeamCursor : juce::MouseCursor::NormalCursor);
}

juce::Rectangle<int> PartInspector::valueArea(std::size_t index) const {
    if (!edit_ || index >= edit_->fields.size()) return {};
    auto area = getLocalBounds().reduced(12, 8);
    area = area.removeFromBottom(editHeight());
    area.removeFromTop(inspectorEditHeader);
    area.removeFromTop(inspectorEditRow * static_cast<int>(index));
    auto row = area.removeFromTop(inspectorEditRow).reduced(0, 2);
    row.removeFromRight(inspectorStepperWidth);
    return row.removeFromRight(inspectorValueWidth);
}

void PartInspector::beginTyping(std::size_t index) {
    if (!edit_ || index >= pending_.size()) return;
    stopTimer();
    sendEdit();
    typing_ = static_cast<int>(index);
    valueEditor_.setText(formatValue(index, pending_[index], false), false);
    valueEditor_.setBounds(valueArea(index));
    valueEditor_.setVisible(true);
    valueEditor_.grabKeyboardFocus();
    valueEditor_.selectAll();
    repaint();
}

void PartInspector::endTyping(bool apply) {
    // Hiding the editor takes its focus, which calls this again.
    if (typing_ < 0) return;
    const auto index = static_cast<std::size_t>(typing_);
    typing_ = -1;
    const auto text = valueEditor_.getText().replaceCharacter(',', '.').trim();
    valueEditor_.setVisible(false);
    if (apply && edit_ && index < pending_.size() && text.containsAnyOf("0123456789")) {
        const auto& field = edit_->fields[index];
        auto typed = text.getDoubleValue();
        // Typed in the viewer's unit: back to the field's own.
        if (field.quantity && units_ != nullptr) {
            const auto& unit = units_->preferences.of(*field.quantity);
            typed = (typed - unit.offset) / unit.scale;
        }
        pending_[index] = std::clamp(typed, field.minimum, field.maximum);
        setEditStatus({});
        updateEditButtons();
        sendEdit();
    }
    repaint();
    if (onDoneTyping) onDoneTyping();
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
constexpr std::array<double, 3> simulationSpeeds { 0.25, 0.5, 1.0 };

[[nodiscard]] double wallSeconds() noexcept { return juce::Time::getMillisecondCounterHiRes() * 1.0e-3; }

/** The setting applies to the All layer; the Gas flow layer exists to show
    the gas, so it falls back to the steady strength instead of nothing. The
    other layers draw no waves. */
[[nodiscard]] ViewSettings::PressureWaves effectivePressureWaves(ViewSettings::PressureWaves setting,
                                                                 SceneLayerMode layer) noexcept {
    if (layer == SceneLayerMode::gasFlow && setting == ViewSettings::PressureWaves::hidden)
        return ViewSettings::PressureWaves::strength;
    if (layer != SceneLayerMode::all && layer != SceneLayerMode::gasFlow) return ViewSettings::PressureWaves::hidden;
    return setting;
}

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

[[nodiscard]] juce::String millimetres(const UnitFormat& unitFormat, double value) {
    if (unitFormat.preferences.of(units::Quantity::length).id != "mm")
        return unitFormat.text(units::Quantity::length, value);
    return juce::String(value, value < 100.0 ? 1 : 0) + " mm";
}

/** The standard atmosphere's pressure at `altitudeM`, and back. */
[[nodiscard]] double pressureKpaAtAltitude(double altitudeM) noexcept {
    return 101.325 * std::pow(1.0 - 2.25577e-5 * altitudeM, 5.25588);
}

[[nodiscard]] double altitudeForPressureKpa(double pressureKpa) noexcept {
    return (1.0 - std::pow(pressureKpa / 101.325, 1.0 / 5.25588)) / 2.25577e-5;
}

/** Moves `field` (and `highField`, the variable profile's, when given) of the
    engine's camshafts and of every bank's by `delta`. */
void shiftCamshafts(EngineConfig& config, double CamshaftConfig::*field, double CamshaftConfig::*highField,
                    double delta) {
    const auto shift = [&](CamshaftConfig& camshafts) {
        camshafts.*field += delta;
        if (highField != nullptr) camshafts.*highField += delta;
    };
    shift(config.camshafts);
    for (auto& bank : config.banks) shift(bank.camshafts);
}

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
    speed_.setOptionTooltip(0, utf8("Simulate four times slower: the engine and its sound slow down"));
    speed_.setOptionTooltip(1, utf8("Simulate twice slower: the engine and its sound slow down"));
    speed_.setOptionTooltip(2, utf8("Simulate in real time"));
    speed_.onChange = [this](int index) {
        if (index >= 0 && onTimeScaleRequested) onTimeScaleRequested(simulationSpeeds[static_cast<std::size_t>(index)]);
    };
    speed_.setSelected(2);
    playback_.setOptionTooltip(0, utf8("The picture follows the simulation"));
    playback_.setOptionTooltip(1, utf8("Stroboscope: the picture turns fifty times slower; the engine and its sound "
                                       "do not change"));
    playback_.setOptionTooltip(2, utf8("Stroboscope: the picture turns 250 times slower; the engine and its sound "
                                       "do not change"));
    playback_.setOptionTooltip(3, utf8("Stroboscope: the picture holds the crank where it is; the engine keeps "
                                       "running"));
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
    inspector_.setUnits(&model_.units);
    inspector_.onApplyEdit = [this](const std::vector<double>& values, bool gradual) { applyEdit(values, gradual); };
    inspector_.onChoice = [this](int index) { applyPartChoice(index); };
    inspector_.onDoneTyping = [this] { if (onClaimKeyboard) onClaimKeyboard(); };

    addChildComponent(cutaway_);
    addChildComponent(legend_);
    addChildComponent(inspector_);
    for (auto* overlay : std::initializer_list<juce::Component*> { &layers_, &views_, &shading_, &speed_,
                                                                   &playback_, &settingsButton_, &cycle_ })
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

void EngineViewport::applyEdit(const std::vector<double>& values, bool gradual) {
    if (!scene_ || selectedPart_ < 0 || values.empty()) return;
    // Every edit has its own field count; a pair below is a length and a
    // diameter, a bore and a stroke...
    const auto first = values[0];
    const auto second = values.size() > 1 ? values[1] : values[0];
    const auto role = scene_->identify(static_cast<std::uint16_t>(selectedPart_)).role;
    const auto identity = scene_->identify(static_cast<std::uint16_t>(selectedPart_));
    if (role == render::PartRole::duct && identity.duct >= 0
        && scene_->ducts()[static_cast<std::size_t>(identity.duct)].kind == render::DuctKind::exhaustComponent)
        applyExhaustEdit(first, second);
    else if (role == render::PartRole::liner || role == render::PartRole::piston)
        applyCylinderEdit(values, gradual);
    else if (role == render::PartRole::rod)
        applyRodEdit(first, gradual);
    else if (partEditor_ && (role == render::PartRole::head || role == render::PartRole::intakeValve
                             || role == render::PartRole::exhaustValve))
        applyCamshaftEdit(values, gradual);
    else if (partEditor_)
        applyPartEdit(values);
}

void EngineViewport::applyCamshaftEdit(const std::vector<double>& values, bool gradual) {
    if (!scene_ || !cylinderEditor_ || !partEditor_ || selectedPart_ < 0) return;
    auto edited = scene_->config();
    sendCylinderEdit(edited, partEditor_(edited, values), gradual);
}

void EngineViewport::applyPartEdit(const std::vector<double>& values) {
    if (!scene_ || !configEditor_ || !partEditor_ || selectedPart_ < 0) return;
    auto edited = scene_->config();
    if (const auto error = partEditor_(edited, values); !error.empty()) {
        inspector_.setEditStatus(juce::String::fromUTF8(error.c_str()));
        return;
    }
    if (const auto invalid = validateEngineConfig(edited)) {
        inspector_.setEditStatus(juce::String::fromUTF8(invalid->c_str()));
        return;
    }
    // The part keeps its index: these edits resize, they add nothing.
    reselect_ = Reselect { 0, 0, selectedPart_ };
    if (!configEditor_(edited)) {
        reselect_.reset();
        inspector_.setEditStatus("The engine did not take the edit.");
    }
}

void EngineViewport::applyPartChoice(int index) {
    if (!scene_ || !configEditor_ || !partChoice_ || selectedPart_ < 0) return;
    auto edited = scene_->config();
    partChoice_(edited, index);
    if (const auto invalid = validateEngineConfig(edited)) {
        inspector_.setEditStatus(juce::String::fromUTF8(invalid->c_str()));
        return;
    }
    reselect_ = partChoiceSelects_ ? Reselect { 0, 0, -1, partChoiceSelects_ } : Reselect { 0, 0, selectedPart_ };
    if (!configEditor_(edited)) {
        reselect_.reset();
        inspector_.setEditStatus("The engine did not take the edit.");
    }
}

void EngineViewport::applyCylinderEdit(const std::vector<double>& values, bool gradual) {
    if (!scene_ || !cylinderEditor_ || selectedPart_ < 0 || values.empty()) return;
    auto edited = scene_->config();
    // The ratio alone when the cylinders are not one size (cylinderEdit).
    const auto ratio = values.back();
    // Only what was edited: a chamber-geometry engine's ratio follows its
    // bore and stroke, and the field still holds the ratio from before.
    std::string error;
    if (const auto size = cylinderSize(edited);
        size && values.size() >= 3U && (values[0] != size->boreMm || values[1] != size->strokeMm))
        error = resizeCylinders(edited, values[0], values[1]);
    if (error.empty() && ratio != scene_->config().cylinders.front().compressionRatio)
        error = setCompressionRatio(edited, ratio);
    sendCylinderEdit(edited, error, gradual);
}

void EngineViewport::applyRodEdit(double rodMm, bool gradual) {
    if (!scene_ || !cylinderEditor_ || selectedPart_ < 0) return;
    auto edited = scene_->config();
    const auto error = resizeConnectingRods(edited, rodMm);
    sendCylinderEdit(edited, error, gradual);
}

void EngineViewport::sendCylinderEdit(const EngineConfig& edited, const std::string& error, bool gradual) {
    if (!error.empty()) {
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
    if (scene_) listenerFrame_ = render::ListenerFrame::of(*scene_);
    soundSources_ = scene_ ? listenerFrame_.sources(*scene_) : AcousticSourcePlacements {};
    ++soundSourcesVersion_;
    const auto& authored = config.acousticObserver;
    const auto spacing = std::hypot(authored.rightMicrophoneM.x - authored.leftMicrophoneM.x,
                                    authored.rightMicrophoneM.y - authored.leftMicrophoneM.y,
                                    authored.rightMicrophoneM.z - authored.leftMicrophoneM.z);
    microphoneSpacingM_ = std::isfinite(spacing) && spacing > 0.01 && spacing < 2.0 ? spacing : 0.36;
    std::vector<juce::String> labels;
    for (const auto& cylinder : config.cylinders) labels.push_back("C" + juce::String(cylinder.id));
    cycle_.setCylinders(std::move(labels), cycleDegreesOf(config.cycle));
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
    if (reselect && scene_ && reselect->role) {
        for (std::size_t part = 0; part < scene_->parts().size(); ++part)
            if (scene_->identify(static_cast<std::uint16_t>(part)).role == *reselect->role) {
                setSelectedPart(static_cast<int>(part));
                break;
            }
    } else if (reselect && scene_ && reselect->part >= 0) {
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
    shared_.pressureWaves = settings_.pressureWaves;
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
    speed_.setVisible(use3d);
    updateLegend();
    if (!use3d) setSelectedPart(-1);
}

void EngineViewport::updateLegend() {
    const auto layer = static_cast<SceneLayerMode>(std::clamp(layers_.selected(), 0, 3));
    const auto waves = effectivePressureWaves(settings_.pressureWaves, layer);
    const auto show = context_.getTargetComponent() != nullptr && pressureScalePa_.load() > 0.0F
        && waves != ViewSettings::PressureWaves::hidden;
    legend_.setStrength(waves == ViewSettings::PressureWaves::strength);
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
    // A field being typed in (a run name...) lets the keys go back to the
    // engine.
    if (onClaimKeyboard) onClaimKeyboard();
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

bool EngineViewport::showInjectors() {
    if (!scene_) return false;
    const auto part = scene_->injectorPart(0);
    if (part >= scene_->parts().size()) return false;
    const auto layer = static_cast<SceneLayerMode>(std::clamp(layers_.selected(), 0, 3));
    if (!scenePartPickable(scene_->parts()[part], layer, shading_.selected() == 0))
        layers_.setSelected(0, juce::sendNotificationSync);
    setSelectedPart(static_cast<int>(part));
    return true;
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
    partEditor_ = {};
    partChoice_ = {};
    partChoiceSelects_.reset();
    std::vector<PartInspector::Row> rows;
    const auto add = [&rows](juce::String label, juce::String value) { rows.push_back({ std::move(label), std::move(value) }); };
    // Bore and stroke are one engine-wide edit, offered on any cylinder.
    const auto cylinderEdit = [this, &config]() -> std::optional<PartInspector::Edit> {
        const auto size = cylinderSize(config);
        if (!cylinderEditor_ || config.cylinders.empty()) return std::nullopt;
        PartInspector::Edit result;
        result.key = std::uint64_t { 1 } << 63U;
        // Cylinders of different sizes are resized one design at a time; the
        // ratio is still one engine-wide value.
        if (size) {
            result.length("Bore", size->boreMm, 1.0, 0.1, 20.0, 200.0);
            result.length("Stroke", size->strokeMm, 1.0, 0.1, 20.0, 200.0);
        } else {
            result.key |= 1U;
        }
        result.value("Compression ratio", config.cylinders.front().compressionRatio, utf8(" : 1"), 1, 0.5, 0.1, 5.0,
                     20.0);
        result.offersRamp = true;
        result.note = size ? "Every cylinder changes, each crank throw at its gas-exchange TDC. A new bore or stroke "
                             "keeps the compression ratio; a higher ratio gives more torque, and knocks sooner."
                           : "Every cylinder takes the ratio: a higher one gives more torque, and knocks sooner. The "
                             "cylinders are not all one size, so bore and stroke are not offered here.";
        return result;
    };
    // A two-stroke's timing is where its ports' top edges sit: a new engine
    // (the restart), every cylinder alike.
    const auto portTimingEdit = [&config]() {
        const auto& ports = config.twoStroke;
        PartInspector::Edit result;
        result.key = std::uint64_t { 11 } << 60U;
        result.heading = "PORT TIMING";
        result.value("Exhaust opens", ports.exhaustPortOpenDegrees, utf8("\xc2\xb0 ATDC"), 0, 2.0, 0.5, 60.0, 120.0);
        result.value("Transfers open", ports.transferPortOpenDegrees, utf8("\xc2\xb0 ATDC"), 0, 2.0, 0.5, 64.0, 150.0);
        result.restarts = true;
        result.note = "Degrees after TDC where the piston uncovers each port; every cylinder changes. A higher "
                      "exhaust port (fewer degrees) gives a longer blowdown and a peakier engine. The transfers open "
                      "at least 4 deg after the exhaust. The engine restarts to take them.";
        return result;
    };
    // The first two values are the exhaust's and the transfers' opening; a
    // power valve keeps how far it lowers the port.
    const auto portTimingEditor = []() {
        return [](EngineConfig& edited, const std::vector<double>& values) {
            if (values.size() < 2U) return std::string("The ports changed meanwhile.");
            auto& ports = edited.twoStroke;
            if (ports.powerValveClosedExhaustPortOpenDegrees != 0.0)
                ports.powerValveClosedExhaustPortOpenDegrees += values[0] - ports.exhaustPortOpenDegrees;
            ports.exhaustPortOpenDegrees = values[0];
            ports.transferPortOpenDegrees = values[1];
            return std::string();
        };
    };
    const auto injectorEdit = [this, &config]() -> std::optional<PartInspector::Edit> {
        if (!configEditor_) return std::nullopt;
        // Carburettors meter every cylinder: no injector is commanded.
        if (!config.intakePaths.empty()
            && std::all_of(config.intakePaths.begin(), config.intakePaths.end(),
                           [](const IntakePathConfig& path) { return path.geometry.carburettor.enabled; }))
            return std::nullopt;
        const auto& injection = config.injection;
        const auto direct = injection.mode == InjectionMode::direct;
        PartInspector::Edit result;
        result.key = std::uint64_t { 2 } << 60U;
        result.heading = "INJECTORS";
        // mg/s to g/min.
        result.value("Flow", injection.injectorFlowMgPerSecond * 0.06, " g/min", 0, 10.0, 1.0, 1.0, 30'000.0);
        result.value("Fuel pressure", injection.railPressureBar, " bar", direct ? 0 : 1, direct ? 10.0 : 0.5,
                     direct ? 1.0 : 0.1, 1.2, 3'000.0);
        // A diesel's start of injection, before firing TDC: the window keeps
        // its length and moves with it.
        const auto diesel = config.fuel == FuelType::diesel;
        if (diesel)
            result.value("Start of injection", 720.0 - std::fmod(injection.startAngleDegrees + 360.0, 720.0) - 360.0,
                         utf8("\xc2\xb0 BTDC"), 0, 2.0, 1.0, -20.0, 60.0);
        result.note = "Every injector changes. The flow is rated at "
            + juce::String(injection.referencePressureBar, direct ? 0 : 1)
            + " bar; the fuel pressure moves it as its square root"
            + (diesel ? juce::String(", and a diesel's mixing burn with it") : juce::String())
            + ". The running engine takes them.";
        partEditor_ = [diesel](EngineConfig& edited, const std::vector<double>& values) {
            const auto flowGPerMin = values[0];
            const auto pressureBar = values[1];
            edited.injection.injectorFlowMgPerSecond = flowGPerMin / 0.06;
            edited.injection.railPressureBar = pressureBar;
            if (diesel && values.size() > 2U) {
                auto& fuelling = edited.injection;
                const auto windowDegrees = std::fmod(fuelling.endAngleDegrees - fuelling.startAngleDegrees + 720.0, 720.0);
                fuelling.startAngleDegrees = std::fmod(720.0 - values[2] + 720.0, 720.0);
                fuelling.endAngleDegrees = std::fmod(fuelling.startAngleDegrees + windowDegrees, 720.0);
            }
            return std::string();
        };
        return result;
    };

    // Fits a turbocharger sized for the engine, or takes it off: a restart.
    // Not offered on a supercharged engine, nor on a two-stroke (its
    // crankcase does the pumping: validation refuses forced induction).
    const auto turboChoice = [this, &config]() -> std::optional<PartInspector::Edit::Choice> {
        const auto& forced = config.forcedInduction;
        if (forced.enabled && forced.type != ForcedInductionType::turbocharger) return std::nullopt;
        if (config.cycle == EngineCycle::twoStroke) return std::nullopt;
        const auto sized = sizedTurbocharger(config);
        partChoice_ = [](EngineConfig& edited, int index) {
            edited = index == 1 ? withTurbocharger(edited) : withoutForcedInduction(edited);
        };
        partChoiceSelects_ = render::PartRole::turbo;
        return PartInspector::Edit::Choice {
            { "No turbo", "Turbo" },
            utf8("Fits a turbocharger sized for this engine, and restarts it: turbine ")
                + juce::String(juce::roundToInt(sized.turbineFlowAreaMm2)) + utf8(" mm\xc2\xb2 on each exhaust path's "
                "collector (at the outlets without one), ")
                + juce::String(sized.pressureRatio - 1.0, 1) + utf8(" bar of boost from ")
                + juce::String(juce::roundToInt(sized.fullBoostRpm)) + utf8(" rpm (a lower boost the higher the "
                "compression). The ignition map stays the engine's own: a high-compression engine may knock at "
                "boost. Or takes the turbo off."),
            forced.enabled ? 1 : 0 };
    };

    // Joins the next exhaust path into `pathId`'s, a restart: a boxer six's
    // two banks made one 6-into-1. Offered only when the join is valid.
    const auto joinEdit = [this, &config](std::uint32_t pathId) -> std::optional<PartInspector::Edit> {
        const auto& paths = config.exhaustPaths;
        if (!configEditor_ || paths.size() < 2U) return std::nullopt;
        const auto kept = std::find_if(paths.begin(), paths.end(),
                                       [pathId](const ExhaustPathConfig& path) { return path.id == pathId; });
        if (kept == paths.end()) return std::nullopt;
        const auto joinedId = std::next(kept) != paths.end() ? std::next(kept)->id : paths.front().id;
        auto trial = config;
        if (!joinExhaustPaths(trial, pathId, joinedId).empty() || validateEngineConfig(trial)) return std::nullopt;
        PartInspector::Edit result;
        result.key = (std::uint64_t { 9 } << 60U) | pathId;
        result.heading = "EXHAUST PATHS";
        result.restarts = true;
        result.choice = PartInspector::Edit::Choice {
            { "Separate", "Joined" },
            "Joins exhaust path " + juce::String(joinedId) + " into path " + juce::String(pathId)
                + ", and restarts: each of its cylinders gets a copy of this path's primary, into this path's "
                  "collector; its own pipes, silencers and outlet go. The collector keeps its size.",
            0 };
        const auto joinedPipes = kept->cylinderIds.size()
            + std::find_if(paths.begin(), paths.end(), [joinedId](const ExhaustPathConfig& path) {
                  return path.id == joinedId;
              })->cylinderIds.size();
        result.note = "Path " + juce::String(joinedId) + " can join this one: one collector for "
            + juce::String(static_cast<int>(joinedPipes)) + " pipes.";
        partChoice_ = [pathId, joinedId](EngineConfig& edited, int index) {
            if (index == 1) (void)joinExhaustPaths(edited, pathId, joinedId);
        };
        // The pipes are redrawn: the ports then show the joined path.
        partChoiceSelects_ = render::PartRole::exhaustPorts;
        return result;
    };

    switch (identity.role) {
    case render::PartRole::block:
        title = "Cylinder block";
        subtitle = juce::String(config.name);
        add("Cylinders", juce::String(static_cast<int>(config.cylinders.size())));
        add("Displacement", juce::String(engineDisplacementLitres(config), 2) + " L");
        // The ambient itself is never shown as a gauge pressure (it would read 0).
        add("Air pressure", model_.units.pressureDifference(config.ambientPressureKpa, 1));
        add("Coolant", model_.units.text(units::Quantity::temperature, state.coolantTemperatureC)
                           + (config.thermal.regulated ? ", held" : state.coolingFanOn ? ", fan on" : ", free"));
        if (configEditor_) {
            edit = PartInspector::Edit {};
            edit->key = std::uint64_t { 8 } << 60U;
            edit->heading = "AIR";
            edit->value("Altitude", std::round(altitudeForPressureKpa(config.ambientPressureKpa)), " m", 0, 250.0,
                        50.0, 0.0, 5'000.0);
            edit->value("Temperature", config.ambientTemperatureC, utf8(" \xc2\xb0""C"), 0, 5.0, 1.0, -30.0, 50.0)
                .quantity = units::Quantity::temperature;
            edit->note = "The air the engine breathes and its exhaust leaves into: the standard atmosphere's "
                         "pressure at that altitude. The running engine takes it.";
            edit->choice = PartInspector::Edit::Choice {
                { "Realistic", "Regulated" },
                utf8("Regulated: coolant and oil held at ")
                    + juce::String(juce::roundToInt(config.thermal.regulatedCoolantTemperatureC))
                    + utf8(" and ") + juce::String(juce::roundToInt(config.thermal.regulatedOilTemperatureC))
                    + utf8(" \xc2\xb0""C, as on a dyno cell, so every run is made warm and alike. Realistic: the "
                           "engine starts at the air temperature, warms up with load and cools by its radiator, "
                           "the air of its speed and its fan (a dyno cell's fan on the dyno)."),
                config.thermal.regulated ? 1 : 0 };
            partChoice_ = [](EngineConfig& edited, int index) { edited.thermal.regulated = index == 1; };
            partEditor_ = [](EngineConfig& edited, const std::vector<double>& values) {
                const auto altitudeM = values[0];
                const auto temperatureC = values[1];
                edited.ambientPressureKpa = pressureKpaAtAltitude(altitudeM);
                edited.ambientTemperatureC = temperatureC;
                return std::string();
            };
        }
        break;
    case render::PartRole::head: {
        title = "Cylinder heads";
        subtitle = juce::String(config.name);
        add("Engine speed", juce::String(juce::roundToInt(state.rpm)) + " rpm");
        const auto degrees = [](double value) { return juce::String(juce::roundToInt(value)) + utf8("\xc2\xb0"); };
        // A two-stroke's head has no valvetrain: its timing is its ports'.
        if (config.cycle == EngineCycle::twoStroke) {
            const auto& ports = config.twoStroke;
            add("Exhaust opens", degrees(ports.exhaustPortOpenDegrees) + " ATDC");
            add("Transfers open", degrees(ports.transferPortOpenDegrees) + " ATDC");
            add("Blowdown", degrees(ports.transferPortOpenDegrees - ports.exhaustPortOpenDegrees));
            add("Compression ratio", juce::String(config.cylinders.front().compressionRatio, 1) + utf8(" : 1"));
            if (cylinderEditor_) {
                edit = portTimingEdit();
                edit->restarts = false;
                edit->value("Compression ratio", config.cylinders.front().compressionRatio, utf8(" : 1"), 1, 0.5, 0.1,
                            5.0, 20.0);
                edit->note = "Degrees after TDC where the piston uncovers each port; every cylinder changes. A port "
                             "change restarts the engine; the compression ratio alone is taken live.";
                const auto ratioNow = config.cylinders.front().compressionRatio;
                partEditor_ = [ports = portTimingEditor(), ratioNow](EngineConfig& edited,
                                                                      const std::vector<double>& values) {
                    if (values.size() < 3U) return std::string("The head changed meanwhile.");
                    auto error = ports(edited, values);
                    if (error.empty() && values.back() != ratioNow) error = setCompressionRatio(edited, values.back());
                    return error;
                };
            }
            break;
        }
        const auto& cams = config.banks.empty() ? config.camshafts : config.banks.front().camshafts;
        add("Lobe separation", degrees(0.5 * (cams.intakeCenterlineDegrees + cams.exhaustCenterlineDegrees)));
        add("Valve overlap", degrees(std::max(0.0, 0.5 * (cams.intakeDurationDegrees + cams.exhaustDurationDegrees)
                                                       - cams.intakeCenterlineDegrees - cams.exhaustCenterlineDegrees)));
        if (cams.variableProfileEnabled)
            add("Profile switch", juce::String(juce::roundToInt(cams.switchRpm)) + " rpm");
        if (cylinderEditor_) {
            PartInspector::Edit timing;
            timing.key = std::uint64_t { 7 } << 60U;
            timing.heading = "CAMS AND CHAMBER";
            timing.value("Intake centreline", cams.intakeCenterlineDegrees, utf8("\xc2\xb0"), 0, 2.0, 1.0, 60.0, 160.0);
            timing.value("Exhaust centreline", cams.exhaustCenterlineDegrees, utf8("\xc2\xb0"), 0, 2.0, 1.0, 60.0,
                         160.0);
            timing.value("Compression ratio", config.cylinders.front().compressionRatio, utf8(" : 1"), 1, 0.5, 0.1,
                         5.0, 20.0);
            timing.offersRamp = true;
            timing.note = "Degrees after TDC for the intake, before it for the exhaust: smaller centrelines, more "
                          "overlap. Every camshaft moves by as much, every cylinder takes the ratio (a higher one "
                          "gives more torque, and knocks sooner). The running engine takes them.";
            edit = timing;
            const auto intakeNow = cams.intakeCenterlineDegrees;
            const auto exhaustNow = cams.exhaustCenterlineDegrees;
            const auto ratioNow = config.cylinders.front().compressionRatio;
            partEditor_ = [=](EngineConfig& edited, const std::vector<double>& values) {
                if (values.size() < 3U) return std::string("The head changed meanwhile.");
                const auto intakeCentreline = values[0];
                const auto exhaustCentreline = values[1];
                shiftCamshafts(edited, &CamshaftConfig::intakeCenterlineDegrees, nullptr, intakeCentreline - intakeNow);
                shiftCamshafts(edited, &CamshaftConfig::exhaustCenterlineDegrees, nullptr,
                               exhaustCentreline - exhaustNow);
                return values[2] != ratioNow ? setCompressionRatio(edited, values[2]) : std::string();
            };
        }
        break;
    }
    case render::PartRole::liner:
        title = "Cylinder liner";
        edit = cylinderEdit();
        if (cylinder != nullptr) {
            add("Bore", millimetres(model_.units, cylinder->boreMm));
            add("Stroke", millimetres(model_.units, cylinder->strokeMm));
            add("Compression ratio", juce::String(cylinder->compressionRatio, 1) + utf8(" : 1"));
        }
        break;
    case render::PartRole::crankshaft:
        title = "Crankshaft";
        subtitle = juce::String(config.name);
        add("Engine speed", juce::String(juce::roundToInt(state.rpm)) + " rpm");
        add("Crank angle", juce::String(juce::roundToInt(displayedAngle_.load())) + utf8("°"));
        add("Crank + flywheel", juce::String(config.rotatingInertiaKgM2, 3) + utf8(" kg·m²"));
        add("With the rods", juce::String(effectiveRotatingInertiaKgM2(config), 3) + utf8(" kg·m²"));
        if (configEditor_ && config.rotatingInertiaKgM2 > 0.0) {
            edit = PartInspector::Edit {};
            edit->key = std::uint64_t { 6 } << 60U;
            edit->heading = "FLYWHEEL";
            edit->value("Inertia", config.rotatingInertiaKgM2, utf8(" kg\xc2\xb7m\xc2\xb2"), 3, 0.01, 0.001, 0.01, 5.0);
            edit->note = "The crankshaft and its flywheel, every shaft scaled alike. A lighter flywheel revs up and "
                         "down faster. The running engine takes it.";
            partEditor_ = [](EngineConfig& edited, const std::vector<double>& values) {
                const auto inertia = values[0];
                const auto scale = inertia / std::max(1.0e-6, edited.rotatingInertiaKgM2);
                for (auto& crankshaft : edited.crankshafts) crankshaft.momentOfInertiaKgM2 *= scale;
                edited.rotatingInertiaKgM2 = inertia;
                return std::string();
            };
        }
        break;
    case render::PartRole::piston:
        title = "Piston";
        edit = cylinderEdit();
        if (cylinder != nullptr) {
            add("Bore", millimetres(model_.units, cylinder->boreMm));
            add("Mass", juce::String(juce::roundToInt(cylinder->pistonMassGrams)) + " g");
        }
        if (live != nullptr) {
            add("Cylinder pressure", model_.units.text(units::Quantity::pressure, live->pressureEstimateBar * 100.0));
            add("Gas temperature", model_.units.text(units::Quantity::temperature, live->gasTemperatureC));
        }
        break;
    case render::PartRole::rod:
        title = "Connecting rod";
        if (const auto rodMm = connectingRodLength(config); cylinderEditor_ && rodMm && cylinder != nullptr) {
            PartInspector::Edit rodEdit;
            rodEdit.key = std::uint64_t { 3 } << 60U;
            rodEdit.length("Length", *rodMm, 5.0, 0.5, std::ceil(12.0 * cylinder->strokeMm) / 10.0, 500.0);
            rodEdit.offersRamp = true;
            rodEdit.note = "Every rod changes, each crank throw at its gas-exchange TDC. "
                           "The compression ratio stays.";
            edit = rodEdit;
        }
        if (cylinder != nullptr) {
            add("Length", millimetres(model_.units, cylinder->connectingRodMm));
            add("Rod / stroke", juce::String(cylinder->connectingRodMm / std::max(1.0, cylinder->strokeMm), 2));
            add("Mass", juce::String(juce::roundToInt(cylinder->connectingRodMassGrams)) + " g");
        }
        break;
    case render::PartRole::powerValve: {
        const auto& ports = config.twoStroke;
        title = "Power valve";
        add("Shut: exhaust opens", juce::String(juce::roundToInt(ports.powerValveClosedExhaustPortOpenDegrees))
                                       + utf8("\xc2\xb0 ATDC"));
        add("Open: exhaust opens", juce::String(juce::roundToInt(ports.exhaustPortOpenDegrees)) + utf8("\xc2\xb0 ATDC"));
        add("Starts opening", juce::String(juce::roundToInt(ports.powerValveOpeningStartRpm)) + " rpm");
        add("Fully open", juce::String(juce::roundToInt(ports.powerValveFullyOpenRpm)) + " rpm");
        add("Now", juce::String(juce::roundToInt(100.0 * state.powerValveOpening)) + " % open");
        if (cylinderEditor_) {
            PartInspector::Edit valveEdit;
            valveEdit.key = std::uint64_t { 12 } << 60U;
            valveEdit.heading = "POWER VALVE";
            valveEdit.value("Shut: exhaust opens", ports.powerValveClosedExhaustPortOpenDegrees, utf8("\xc2\xb0 ATDC"),
                            0, 1.0, 0.5, ports.exhaustPortOpenDegrees + 1.0, ports.transferPortOpenDegrees - 4.0);
            valveEdit.value("Starts opening", ports.powerValveOpeningStartRpm, " rpm", 0, 250.0, 50.0, 0.0, 30'000.0);
            valveEdit.value("Fully open", ports.powerValveFullyOpenRpm, " rpm", 0, 250.0, 50.0, 100.0, 30'000.0);
            valveEdit.restarts = true;
            valveEdit.note = "Shut, the blade lowers the port's roof so the exhaust opens later: more of the stroke "
                             "at low speed. Its servo lifts it between the two speeds. Every cylinder changes; the "
                             "engine restarts to take it. The exhaust port's own choice takes it off.";
            edit = valveEdit;
            partEditor_ = [](EngineConfig& edited, const std::vector<double>& values) {
                if (values.size() < 3U) return std::string("The power valve changed meanwhile.");
                if (values[2] < values[1] + 100.0)
                    return std::string("The power valve is fully open at least 100 rpm after it starts to open.");
                auto& ports = edited.twoStroke;
                ports.powerValveClosedExhaustPortOpenDegrees = values[0];
                ports.powerValveOpeningStartRpm = values[1];
                ports.powerValveFullyOpenRpm = values[2];
                return std::string();
            };
        }
        break;
    }
    case render::PartRole::intakeValve:
    case render::PartRole::exhaustValve: {
        const auto intake = identity.role == render::PartRole::intakeValve;
        // A two-stroke's parts here are its ports (EngineModel3D); no cam.
        if (config.cycle == EngineCycle::twoStroke) {
            title = intake ? "Transfer ports" : "Exhaust port";
            const auto opens = intake ? config.twoStroke.transferPortOpenDegrees
                                      : config.twoStroke.exhaustPortOpenDegrees;
            add("Opens", juce::String(juce::roundToInt(opens)) + utf8("\xc2\xb0 ATDC"));
            add("Open for", juce::String(juce::roundToInt(360.0 - 2.0 * opens)) + utf8("\xc2\xb0"));
            if (cylinder != nullptr)
                add("Width", millimetres(model_.units, intake ? transferPortWidthMm(config.twoStroke, *cylinder)
                                                              : exhaustPortWidthMm(config.twoStroke, *cylinder)));
            if (cylinderEditor_ && cylinder != nullptr) {
                edit = portTimingEdit();
                edit->key |= intake ? 1U : 2U;
                edit->length(intake ? "Transfer width" : "Exhaust width",
                             intake ? transferPortWidthMm(config.twoStroke, *cylinder)
                                    : exhaustPortWidthMm(config.twoStroke, *cylinder),
                             1.0, 0.5, 5.0, 500.0);
                const auto widthNow = intake ? transferPortWidthMm(config.twoStroke, *cylinder)
                                             : exhaustPortWidthMm(config.twoStroke, *cylinder);
                const auto cylinderPort = intake && config.twoStroke.intake == TwoStrokeIntake::cylinderPort;
                const auto intakeWidthNow = intakePortWidthMm(config.twoStroke, *cylinder);
                if (cylinderPort) {
                    edit->value("Intake opens", config.twoStroke.intakePortOpenDegreesBtdc, utf8("\xc2\xb0 BTDC"), 0, 2.0,
                                0.5, 30.0, 110.0);
                    edit->length("Intake width", intakeWidthNow, 1.0, 0.5, 5.0, 500.0);
                    edit->note += " The intake port is uncovered by the skirt as far either side of TDC as it opens "
                                  "before it.";
                }
                if (!intake) {
                    // Fitting a power valve, or taking it off, restarts the
                    // engine; its settings are then on its blade.
                    const auto fitted = hasPowerValve(config.twoStroke);
                    edit->choice = PartInspector::Edit::Choice {
                        { "No power valve", "Power valve" },
                        "A blade in the port's roof that lowers it at low speed, so the exhaust opens later and keeps "
                        "more of the stroke, and lifts as the speed rises. Fitted shut 12 deg later than the port, "
                        "opening from 60 % of the redline to fully open at 85 %, as the designer fits one; click its "
                        "blade to set it.",
                        fitted ? 1 : 0 };
                    partChoice_ = [](EngineConfig& edited, int index) {
                        auto& ports = edited.twoStroke;
                        if (index == 1 && !hasPowerValve(ports)) {
                            ports.powerValveClosedExhaustPortOpenDegrees = std::min(ports.exhaustPortOpenDegrees + 12.0,
                                ports.transferPortOpenDegrees - 4.0);
                            ports.powerValveOpeningStartRpm = 0.60 * edited.redlineRpm;
                            ports.powerValveFullyOpenRpm = 0.85 * edited.redlineRpm;
                        } else if (index == 0) {
                            ports.powerValveClosedExhaustPortOpenDegrees = 0.0;
                            ports.powerValveOpeningStartRpm = 0.0;
                            ports.powerValveFullyOpenRpm = 0.0;
                        }
                    };
                    partChoiceSelects_ = fitted ? render::PartRole::exhaustValve : render::PartRole::powerValve;
                }
                partEditor_ = [ports = portTimingEditor(), intake, widthNow, cylinderPort, intakeWidthNow](
                                  EngineConfig& edited, const std::vector<double>& values) {
                    if (values.size() < (cylinderPort ? 5U : 3U)) return std::string("The ports changed meanwhile.");
                    auto error = ports(edited, values);
                    // Unchanged, a width derived from the bore stays derived.
                    if (error.empty() && values[2] != widthNow)
                        (intake ? edited.twoStroke.transferPortWidthMm : edited.twoStroke.exhaustPortWidthMm) = values[2];
                    if (error.empty() && cylinderPort) {
                        edited.twoStroke.intakePortOpenDegreesBtdc = values[3];
                        if (values[4] != intakeWidthNow) edited.twoStroke.intakePortWidthMm = values[4];
                    }
                    return error;
                };
            }
            if (intake && config.twoStroke.intake == TwoStrokeIntake::cylinderPort) {
                title = "Transfer and intake ports";
                add("Intake opens", juce::String(juce::roundToInt(config.twoStroke.intakePortOpenDegreesBtdc))
                                        + utf8("\xc2\xb0 BTDC"));
                add("Intake open for",
                    juce::String(juce::roundToInt(2.0 * config.twoStroke.intakePortOpenDegreesBtdc)) + utf8("\xc2\xb0"));
                add("Reed", config.twoStroke.intakePortReed ? "In the port" : "None (piston port)");
            }
            if (!intake && hasPowerValve(config.twoStroke))
                add("Power valve", juce::String(juce::roundToInt(100.0 * state.powerValveOpening)) + " % open");
            break;
        }
        title = intake ? "Intake valve" : "Exhaust valve";
        const auto& cams = scene_->camshaftsOf(cylinderIndex);
        if (cylinderEditor_) {
            PartInspector::Edit camEdit;
            camEdit.key = (std::uint64_t { 7 } << 60U) | (intake ? 1U : 2U);
            camEdit.heading = intake ? "INTAKE CAM" : "EXHAUST CAM";
            camEdit.value("Duration", intake ? cams.intakeDurationDegrees : cams.exhaustDurationDegrees,
                          utf8("\xc2\xb0"), 0, 5.0, 1.0, 120.0, 360.0);
            camEdit.length("Lift", intake ? cams.intakeLiftMm : cams.exhaustLiftMm, 0.5, 0.1, 1.0, 20.0).decimals = 1;
            camEdit.offersRamp = true;
            camEdit.note = "Every camshaft moves by as much, its high profile too. The running engine takes it.";
            edit = camEdit;
            const auto durationField = intake ? &CamshaftConfig::intakeDurationDegrees
                                              : &CamshaftConfig::exhaustDurationDegrees;
            const auto liftField = intake ? &CamshaftConfig::intakeLiftMm : &CamshaftConfig::exhaustLiftMm;
            const auto highDurationField = intake ? &CamshaftConfig::highIntakeDurationDegrees
                                                  : &CamshaftConfig::highExhaustDurationDegrees;
            const auto highLiftField = intake ? &CamshaftConfig::highIntakeLiftMm : &CamshaftConfig::highExhaustLiftMm;
            const auto durationNow = cams.*durationField;
            const auto liftNow = cams.*liftField;
            partEditor_ = [=](EngineConfig& edited, const std::vector<double>& values) {
                const auto duration = values[0];
                const auto lift = values[1];
                shiftCamshafts(edited, durationField, highDurationField, duration - durationNow);
                shiftCamshafts(edited, liftField, highLiftField, lift - liftNow);
                return std::string();
            };
        }
        add("Lift", millimetres(model_.units, intake ? cams.intakeLiftMm : cams.exhaustLiftMm));
        add("Duration", juce::String(juce::roundToInt(intake ? cams.intakeDurationDegrees : cams.exhaustDurationDegrees))
                            + utf8("°"));
        add("Centreline", juce::String(juce::roundToInt(intake ? cams.intakeCenterlineDegrees : cams.exhaustCenterlineDegrees))
                              + utf8("°"));
        if (live != nullptr)
            add("Lift now", millimetres(model_.units, intake ? live->intakeValveLiftMm : live->exhaustValveLiftMm));
        break;
    }
    case render::PartRole::combustion:
        title = "Combustion";
        if (config.injection.mode == InjectionMode::direct) {
            edit = injectorEdit();
            add("Injection", "Direct");
        }
        if (live != nullptr) {
            add("Burned fraction", juce::String(juce::roundToInt(100.0 * live->burnedFraction)) + " %");
            add("Flame speed", juce::String(live->flameSpeedMps, 1) + " m/s");
            add("Misfiring", live->misfiring ? "yes" : "no");
        }
        break;
    case render::PartRole::intakePorts:
        title = "Intake ports";
        if (config.injection.mode != InjectionMode::direct) {
            edit = injectorEdit();
            add("Injection", "Port");
        }
        break;
    case render::PartRole::injector: {
        const auto direct = config.injection.mode == InjectionMode::direct;
        title = "Injector";
        edit = injectorEdit();
        add("Injection", direct ? "Direct" : "Port");
        add("Window", juce::String(juce::roundToInt(config.injection.startAngleDegrees)) + utf8("° to ")
                          + juce::String(juce::roundToInt(config.injection.endAngleDegrees)) + utf8("°"));
        if (live != nullptr) {
            add("Duty cycle", juce::String(100.0 * live->injectorDutyCycle, 1) + " %");
            add("Fuel per cycle", juce::String(live->meteredFuelMgPerCycle, 1) + " mg");
            if (!direct) add("Port wall film", juce::String(live->portLiquidFilmFuelMg, 1) + " mg");
        }
        break;
    }
    case render::PartRole::exhaustPorts:
        title = "Exhaust ports";
        if (live != nullptr) add("Exhaust gas", model_.units.text(units::Quantity::temperature, live->exhaustTemperatureC));
        if (cylinder != nullptr)
            for (const auto& path : config.exhaustPaths)
                if (std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinder->id) != path.cylinderIds.end()) {
                    add("Exhaust path", juce::String(path.id) + ", " + juce::String(static_cast<int>(path.cylinderIds.size()))
                                            + (path.cylinderIds.size() == 1U ? " cylinder" : " cylinders"));
                    edit = joinEdit(path.id);
                }
        // The stubs of every cylinder are one part: the paths, the first
        // offered the next one.
        if (cylinder == nullptr && !config.exhaustPaths.empty()) {
            add("Exhaust paths", juce::String(static_cast<int>(config.exhaustPaths.size())));
            edit = joinEdit(config.exhaustPaths.front().id);
        }
        break;
    case render::PartRole::afterfire:
        title = "Afterfire";
        subtitle = "Flame at the outlet";
        add("Heat release", model_.units.text(units::Quantity::power, state.exhaustAfterfireHeatReleaseKw, 2));
        break;
    case render::PartRole::turbo: {
        const auto& forced = config.forcedInduction;
        const auto krpm = [](double rpm) { return juce::String(rpm / 1'000.0, 1) + " krpm"; };
        const auto area = [](double mm2) { return juce::String(juce::roundToInt(mm2)) + utf8(" mm²"); };
        const auto& drawn = scene_->turbos();
        const auto& clicked = identity.turbo >= 0 && static_cast<std::size_t>(identity.turbo) < drawn.size()
            ? drawn[static_cast<std::size_t>(identity.turbo)] : scene_->turbo();
        title = drawn.size() > 1U ? "Turbocharger " + juce::String(static_cast<int>(clicked.unit) + 1)
                                  : juce::String("Turbocharger");
        subtitle = "On exhaust path " + juce::String(clicked.pathId);
        const auto units = std::min<std::size_t>(state.turbochargerCount, maximumTurbochargers);
        if (units > 1U) {
            juce::String speeds;
            juce::String shares;
            auto totalFlow = 0.0;
            for (std::size_t index = 0; index < units; ++index)
                totalFlow += state.turbochargers[index].compressorMassFlowKgPerSecond;
            for (std::size_t index = 0; index < units; ++index) {
                const auto& unit = state.turbochargers[index];
                speeds << (index > 0 ? " / " : "") << juce::String(unit.shaftSpeedRpm / 1'000.0, 0);
                shares << (index > 0 ? " / " : "")
                       << juce::roundToInt(totalFlow > 0.0 ? 100.0 * unit.compressorMassFlowKgPerSecond / totalFlow : 0.0);
            }
            add("Turbochargers", juce::String(static_cast<int>(units)) + " on one charge");
            add("Shaft speeds", speeds + " krpm");
            add("Air shares", shares + " %");
        } else {
            add("Shaft speed", krpm(state.forcedInductionShaftSpeedRpm));
        }
        if (units > 0U) {
            // The compressor map: below 0.4 x its design flow coefficient it
            // surges, from 1.5 x it chokes.
            const auto ratio = state.turbochargers[0].compressorFlowCoefficientRatio;
            add(units > 1U ? "Map position (turbo 1)" : "Map position",
                juce::String(ratio, 2) + utf8(" \xc3\x97 design")
                    + (ratio < 0.4 ? ", surge" : ratio >= 1.5 ? ", choke" : ""));
        }
        add("Pressure ratio", juce::String(state.boostPressureRatio, 2) + " (target " + juce::String(forced.pressureRatio, 2) + ")");
        if (forced.intercoolerEffectiveness > 0.0)
            add("Intercooler", juce::String(juce::roundToInt(100.0 * forced.intercoolerEffectiveness)) + " %");
        add("Turbine outlet", model_.units.text(units::Quantity::pressure, state.turbineOutletPressureKpa));
        add("Wastegate", juce::String(juce::roundToInt(100.0 * state.wastegateOpening)) + " % open");
        add("Turbine / compressor", model_.units.value(units::Quantity::power, state.turbinePowerKw) + " / "
                                        + model_.units.text(units::Quantity::power, state.compressorPowerKw));
        add("Design shaft speed", krpm(forced.designShaftSpeedRpm));
        add("Turbine flow area", area(forced.turbineFlowAreaMm2));
        add("Wastegate flow area", area(forced.wastegateFlowAreaMm2));
        add("Blades (compressor / turbine)", forced.compressorBladeCount > 0U && forced.turbineBladeCount > 0U
                ? juce::String(static_cast<int>(forced.compressorBladeCount)) + " / "
                    + juce::String(static_cast<int>(forced.turbineBladeCount))
                : juce::String("not set"));
        if (forced.compressorInducerDiameterMm > 0.0 && forced.turbineExducerDiameterMm > 0.0)
            add("Inducer / exducer", juce::String(juce::roundToInt(forced.compressorInducerDiameterMm)) + " / "
                                         + millimetres(model_.units, forced.turbineExducerDiameterMm));
        note = "The turbine is a face of the exhaust network at the collector (at the outlets on a path without "
               "one): what follows it is its cold side, and its shaft is driven by the pressure and flow across "
               "it. The compressor raises the intake pressure; the charge piping is not drawn.";
        if (configEditor_) {
            edit = PartInspector::Edit {};
            edit->key = std::uint64_t { 3 } << 60U;
            edit->heading = "TUNE";
            edit->value("Wastegate opens at", forced.wastegatePressureRatio, utf8(" \xc3\x97"), 2, 0.05, 0.01, 1.0, 4.0);
            edit->value(forced.turbochargerCount > 1U ? "Turbine flow area, each" : "Turbine flow area",
                        forced.turbineFlowAreaMm2, utf8(" mm\xc2\xb2"), 0, 50.0, 5.0, 20.0, 20'000.0);
            edit->value("Intercooler", 100.0 * forced.intercoolerEffectiveness, " %", 0, 5.0, 1.0, 0.0, 95.0);
            edit->value("Turbochargers", static_cast<double>(forced.turbochargerCount), "", 0, 1.0, 1.0, 1.0,
                        static_cast<double>(maximumTurbochargers));
            // Which turbocharger this path's turbine drives, when there is a choice.
            const auto pathId = clicked.pathId;
            const auto path = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
                [pathId](const ExhaustPathConfig& candidate) { return candidate.id == pathId; });
            const auto pathIndex = static_cast<std::size_t>(std::distance(config.exhaustPaths.begin(), path));
            const auto choosesUnit = forced.turbochargerCount > 1U && path != config.exhaustPaths.end();
            if (choosesUnit) {
                const auto unit = path->turbocharger >= 0
                    ? static_cast<std::size_t>(path->turbocharger) : pathIndex % forced.turbochargerCount;
                edit->value("Path " + juce::String(pathId) + " drives turbo", static_cast<double>(unit + 1U), "", 0,
                            1.0, 1.0, 1.0, static_cast<double>(forced.turbochargerCount));
            }
            edit->note = utf8("The wastegate pressure ratio limits the boost; the rated ratio keeps its distance to "
                              "it (the ECU's boost table moves with it). The intercooler removes that share of the "
                              "compressor's heat. The running engine takes these. Another turbocharger count "
                              "shares the same total turbo out between identical units, and a path given another "
                              "turbo: both restart the engine.");
            partEditor_ = [choosesUnit, pathIndex](EngineConfig& edited, const std::vector<double>& values) {
                const auto wastegate = values[0];
                const auto turbineAreaMm2 = values[1];
                const auto intercoolerPercent = values[2];
                const auto count = static_cast<std::uint32_t>(std::lround(values[3]));
                auto& turbo = edited.forcedInduction;
                const auto margin = turbo.pressureRatio - turbo.wastegatePressureRatio;
                turbo.wastegatePressureRatio = wastegate;
                turbo.pressureRatio = std::max(1.0, wastegate + margin);
                turbo.turbineFlowAreaMm2 = turbineAreaMm2;
                turbo.intercoolerEffectiveness = std::clamp(intercoolerPercent / 100.0, 0.0, 0.95);
                if (choosesUnit && values.size() > 4U && pathIndex < edited.exhaustPaths.size()
                    && count == turbo.turbochargerCount) {
                    // Written only when it changes: an authored index equal to
                    // the default is still another config, and a restart.
                    auto& editedPath = edited.exhaustPaths[pathIndex];
                    const auto now = editedPath.turbocharger >= 0
                        ? editedPath.turbocharger : static_cast<std::int32_t>(pathIndex % turbo.turbochargerCount);
                    const auto wanted = static_cast<std::int32_t>(std::lround(values[4])) - 1;
                    if (wanted != now) editedPath.turbocharger = wanted;
                }
                if (count != turbo.turbochargerCount) edited = withTurbochargerCount(std::move(edited), count);
                return std::string();
            };
            edit->choice = turboChoice();
        }
        break;
    }
    case render::PartRole::supercharger: {
        const auto& forced = config.forcedInduction;
        title = "Supercharger";
        subtitle = "On intake path " + juce::String(scene_->supercharger().pathId);
        add("Drive ratio", juce::String(forced.superchargerDriveRatio, 2) + utf8(" : 1"));
        add("Impeller speed", juce::String(state.rpm * forced.superchargerDriveRatio / 1'000.0, 1) + " krpm");
        add("Pressure ratio", juce::String(state.boostPressureRatio, 2) + " (target " + juce::String(forced.pressureRatio, 2) + ")");
        add("Compressor", model_.units.text(units::Quantity::power, state.compressorPowerKw));
        if (forced.compressorInducerDiameterMm > 0.0) add("Inducer", millimetres(model_.units, forced.compressorInducerDiameterMm));
        if (forced.compressorBladeCount > 0U) add("Blades", juce::String(static_cast<int>(forced.compressorBladeCount)));
        note = "The gas solver has no compressor in its network: the supercharger raises the intake pressure by its "
               "ratio, geared to the crank. Drawn between the airbox and the throttle, with its charge pipe.";
        if (configEditor_) {
            edit = PartInspector::Edit {};
            edit->key = std::uint64_t { 4 } << 60U;
            edit->heading = "TUNE";
            edit->value("Pressure ratio", forced.pressureRatio, utf8(" \xc3\x97"), 2, 0.05, 0.01, 1.0, 4.0);
            edit->value("Drive ratio", forced.superchargerDriveRatio, utf8(" : 1"), 2, 0.1, 0.01, 0.2, 20.0);
            edit->note = "The running engine takes both.";
            partEditor_ = [](EngineConfig& edited, const std::vector<double>& values) {
                const auto ratio = values[0];
                const auto drive = values[1];
                edited.forcedInduction.pressureRatio = ratio;
                edited.forcedInduction.superchargerDriveRatio = drive;
                return std::string();
            };
        }
        break;
    }
    case render::PartRole::gearbox: {
        const auto& transmission = config.transmission;
        const auto gears = static_cast<int>(transmission.gearRatios.size());
        title = "Gearbox";
        const auto style = transmission.shiftStyle;
        subtitle = style == ShiftStyle::dualClutch ? juce::String("Dual clutch")
            : style == ShiftStyle::cvt ? juce::String(transmission.centrifugalClutch
                ? "CVT, centrifugal clutch" : "CVT, wet start clutch")
            : style == ShiftStyle::manual ? juce::String("Manual, clutch and lift")
            : juce::String("Sequential") + (transmission.quickshifter ? ", quickshifter" : "")
                + (transmission.autoBlipper ? ", auto-blipper" : "");
        const auto gear = state.gear;
        const auto variable = style == ShiftStyle::cvt;
        add("Gear", gear == -2 ? juce::String("Reverse") : gear < 0 || gear >= gears ? juce::String("Neutral")
                : variable ? juce::String("Drive") : juce::String(gear + 1) + " of " + juce::String(gears));
        if (variable && gear >= 0) {
            add("Ratio", juce::String(state.gearboxRatio, 3) + utf8(" : 1, ")
                             + juce::String(state.gearboxRatio * transmission.finalDriveRatio, 2)
                             + utf8(" : 1 overall"));
        } else if (gear >= 0 && gear < gears) {
            const auto ratio = transmission.gearRatios[static_cast<std::size_t>(gear)];
            add("Ratio", juce::String(ratio, 3) + utf8(" : 1, ") + juce::String(ratio * transmission.finalDriveRatio, 2)
                             + utf8(" : 1 overall"));
        }
        add("Input shaft", juce::String(juce::roundToInt(gear == -1 ? state.rpm : state.rpm - state.clutchSlipRpm)) + " rpm");
        add("Clutch slip", juce::String(juce::roundToInt(gear == -1 ? 0.0 : state.clutchSlipRpm)) + " rpm");
        add("Clutch capacity", juce::String(juce::roundToInt(transmission.maxClutchTorqueNm)) + utf8(" N\xc2\xb7m"));
        add("Road speed", model_.units.text(units::Quantity::speed, state.vehicleSpeedMps));
        if (state.shiftInProgress) add("Shifting", juce::String(juce::roundToInt(state.shiftProgress * 100.0)) + " %");
        note = "Each pair turns at its own ratio; the blue collar locks the engaged gear to the output shaft. "
               "A shift is made the way its style makes it: clutch and lift, an ignition cut, a blip on the way "
               "down, or one clutch handing over to the other.";
        if (configEditor_) {
            edit = PartInspector::Edit {};
            edit->key = std::uint64_t { 9 } << 60U;
            edit->heading = "GEARBOX";
            edit->value(style == ShiftStyle::dualClutch ? "Hand-over" : "Clutch shift",
                        std::round(transmission.shiftDurationSeconds * 1'000.0), " ms", 0, 20.0, 5.0, 10.0, 3'000.0);
            edit->value("Ignition cut", std::round(transmission.quickshifterCutSeconds * 1'000.0), " ms", 0, 10.0,
                        1.0, 10.0, 500.0);
            edit->value("Final drive", transmission.finalDriveRatio, utf8(" : 1"), 3, 0.1, 0.01, 0.5, 20.0);
            edit->value("Clutch capacity", std::round(transmission.maxClutchTorqueNm), utf8(" N\xc2\xb7m"), 0, 10.0, 1.0,
                        10.0, 10'000.0);
            for (int index = 0; index < gears; ++index)
                edit->value("Gear " + juce::String(index + 1),
                            transmission.gearRatios[static_cast<std::size_t>(index)], utf8(" : 1"), 3, 0.05, 0.005,
                            0.05, 10.0);
            edit->note = "Clutch shift: the time a shift made with the clutch takes (a dual clutch's hand-over). "
                         "Ignition cut: the quickshifter's, on the way up. Ratios: input over output, the final "
                         "drive multiplies each; the next gear must be taller (a smaller ratio). Clutch capacity: "
                         "the most torque the engaged clutch carries before it slips (hot, it carries less). The "
                         "running engine takes them, in the gear it is in.";
            const auto index = style == ShiftStyle::manual ? 0 : style == ShiftStyle::dualClutch ? 3
                : style == ShiftStyle::cvt ? (transmission.centrifugalClutch ? 5 : 4)
                : transmission.autoBlipper ? 2 : 1;
            edit->choice = PartInspector::Edit::Choice {
                { "Manual", "Quickshifter", "QS + blipper", "Dual clutch", "CVT", "CVT, scooter" },
                "How a gear change is made. Manual: the clutch dips and the throttle lifts. Quickshifter: a "
                "sequential box, clutch left in, the ignition cut while the dogs change on the way up. "
                "QS + blipper: the same, and the throttle blipped to the lower gear's speed on the way down. "
                "Dual clutch: one clutch hands the drive to the other with no gap, and blips down. "
                "CVT: a belt variator between the first and the last gear's ratios, holding the engine at a "
                "speed set by the pedal; it starts on a wet clutch that creeps (CVT) or a centrifugal one that "
                "idles free (CVT, scooter).",
                index };
            partChoice_ = [](EngineConfig& edited, int choice) {
                auto& box = edited.transmission;
                box.shiftStyle = choice == 0 ? ShiftStyle::manual
                    : choice == 3 ? ShiftStyle::dualClutch
                    : choice >= 4 ? ShiftStyle::cvt : ShiftStyle::sequential;
                if (choice == 1 || choice == 2) box.quickshifter = true;
                if (choice >= 1 && choice <= 3) box.autoBlipper = choice != 1;
                if (choice >= 4) box.centrifugalClutch = choice == 5;
            };
            partEditor_ = [](EngineConfig& edited, const std::vector<double>& values) {
                const auto shiftMs = values[0];
                const auto cutMs = values[1];
                auto& box = edited.transmission;
                box.shiftDurationSeconds = shiftMs / 1'000.0;
                box.quickshifterCutSeconds = cutMs / 1'000.0;
                if (values.size() != 4 + box.gearRatios.size()) return std::string("The gearbox changed meanwhile.");
                box.finalDriveRatio = values[2];
                box.maxClutchTorqueNm = values[3];
                for (std::size_t gearIndex = 0; gearIndex < box.gearRatios.size(); ++gearIndex) {
                    box.gearRatios[gearIndex] = values[4 + gearIndex];
                    // Each gear taller than the one below: the scheduler and
                    // the shift protection count on it.
                    if (gearIndex > 0 && box.gearRatios[gearIndex] >= box.gearRatios[gearIndex - 1])
                        return "Gear " + std::to_string(gearIndex + 1) + " must be taller (a smaller ratio) than gear "
                            + std::to_string(gearIndex) + ".";
                }
                return std::string();
            };
        }
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
        // A two-stroke's runner is its transfer duct, its throttle the reed
        // block and carburettor, its inlet duct the boot to the airbox.
        if (config.cycle == EngineCycle::twoStroke) {
            if (duct.kind == render::DuctKind::intakeRunner) title = "Transfer duct";
            else if (duct.kind == render::DuctKind::intakeThrottle) title = "Reed block";
            else if (duct.kind == render::DuctKind::intakeInletDuct) title = "Intake boot";
        }
        const auto exhaust = duct.kind == render::DuctKind::exhaustComponent || duct.kind == render::DuctKind::exhaustFeeder;
        if (duct.kind == render::DuctKind::exhaustFeeder) edit = joinEdit(duct.pathId);
        if (duct.kind == render::DuctKind::exhaustComponent && configEditor_) {
            if (const auto size = exhaustComponentSize(config, duct.pathId, duct.elementId)) {
                edit = PartInspector::Edit {};
                edit->key = (static_cast<std::uint64_t>(duct.pathId) << 32U) | duct.elementId;
                edit->length("Length", size->lengthMm).editable = size->lengthEditable;
                edit->length("Diameter", size->diameterMm, 1.0, 0.1, 10.0, 400.0);
                if (size->sharedByPrimaries)
                    edit->note = "The configuration gives the primaries of a path one size: all of them change.";
                edit->choice = turboChoice();
            }
        }
        std::optional<IntakePart> intakePart;
        if (duct.kind == render::DuctKind::intakeRunner) intakePart = IntakePart::runner;
        else if (duct.kind == render::DuctKind::intakePlenum) intakePart = IntakePart::plenum;
        else if (duct.kind == render::DuctKind::intakeThrottle) intakePart = IntakePart::throttle;
        if (intakePart && configEditor_) {
            if (const auto size = intakePartSize(config, duct.pathId, *intakePart, cylinderIndex)) {
                const auto part = *intakePart;
                const auto pathId = duct.pathId;
                edit = PartInspector::Edit {};
                edit->key = (std::uint64_t { 5 } << 60U) | (static_cast<std::uint64_t>(pathId) << 32U)
                    | (static_cast<std::uint64_t>(part) << 16U) | (part == IntakePart::runner ? cylinderIndex : 0U);
                if (part == IntakePart::runner) {
                    edit->length("Length", size->first, 10.0, 1.0, 10.0, 2'000.0);
                    edit->length("Diameter", size->second, 1.0, 0.1, 10.0, 200.0);
                    if (size->sharedByRunners)
                        edit->note = "The configuration gives the runners of a path one size: all of them change.";
                } else if (part == IntakePart::plenum) {
                    edit->value("Volume", size->first, " L", 2, 0.1, 0.01, 0.05, 40.0);
                } else {
                    edit->length("Bore", size->first, 1.0, 0.1, 10.0, 150.0);
                }
                // A carburettor is its throttle bore: its jets are tuned there.
                const auto tune = part == IntakePart::throttle ? carburettorTune(config, pathId) : std::nullopt;
                auto coefficient = 0.80;
                for (const auto& path : config.intakePaths)
                    if (path.id == pathId) coefficient = path.geometry.carburettor.jetDischargeCoefficient;
                // A jet as the hole Dellorto and Keihin stamp, hundredths of a
                // millimetre; the configuration keeps the flow number.
                const auto hole = [coefficient](double flowNumber) {
                    return jetHoleHundredthsMm(flowNumber, coefficient);
                };
                if (tune) {
                    title = "Carburettor";
                    edit->heading = "JETTING";
                    edit->value("Main jet", std::round(hole(tune->mainJetFlowNumber)), " hole", 0, 2.0, 1.0,
                                std::ceil(hole(50.0)), std::floor(hole(1'000.0)));
                    edit->value("Pilot jet", std::round(hole(tune->pilotJetFlowNumber)), " hole", 0, 2.0, 1.0,
                                std::ceil(hole(10.0)), std::floor(hole(150.0)));
                    edit->value("Needle clip", tune->needleClipPosition, {}, 0, 1.0, 1.0, 1.0, 5.0);
                    edit->value("Air screw", tune->airScrewTurns, " turns out", 2, 0.25, 0.125, 0.0, 5.0);
                    add("Main jet, Mikuni", juce::String(tune->mainJetFlowNumber, 0));
                    add("Pilot jet, Mikuni", juce::String(tune->pilotJetFlowNumber, 1));
                    edit->note = "Jets as the hole a Dellorto or Keihin jet is stamped with, in hundredths of a "
                                 "millimetre; the Mikuni rows are the flow number (cc/min at one metre of head) "
                                 "the same hole passes. The main meters from about a quarter throttle up, the "
                                 "pilot at and just off idle, the needle between them (clip 1 richest, 5 "
                                 "leanest); turning the air screw out leans the pilot. A new bore scales the "
                                 "main jet with the throat's area, so the mixture holds. Every carburettor "
                                 "changes alike. The running engine takes them.";
                }
                partEditor_ = [part, pathId, cylinderIndex, carburetted = tune.has_value(), hole, coefficient](
                                  EngineConfig& edited, const std::vector<double>& values) {
                    if (part == IntakePart::runner)
                        return resizeIntakePart(edited, pathId, part, cylinderIndex, values[0], values[1]);
                    if (!carburetted || values.size() < 5U)
                        return resizeIntakePart(edited, pathId, part, cylinderIndex, values[0]);
                    // A two-stroke has one per cylinder, bored and jetted alike.
                    const auto before = carburettorTune(edited, pathId);
                    std::string error;
                    const auto paths = edited.intakePaths;
                    for (const auto& path : paths)
                        if (carburettorTune(edited, path.id))
                            if (error = resizeIntakePart(edited, path.id, part, cylinderIndex, values[0]); !error.empty())
                                return error;
                    const auto after = carburettorTune(edited, pathId);
                    if (!before || !after) return std::string { "The carburettor is gone." };
                    // A hole left as shown keeps its flow number, the main's
                    // as the new bore scaled it.
                    const auto jet = [&](double shown, double was, double now) {
                        return std::abs(shown - std::round(hole(was))) < 0.5 ? now
                                                                              : jetFlowNumberForHole(shown, coefficient);
                    };
                    const CarburettorTune tuned { jet(values[1], before->mainJetFlowNumber, after->mainJetFlowNumber),
                                                  jet(values[2], before->pilotJetFlowNumber, after->pilotJetFlowNumber),
                                                  values[3], values[4] };
                    for (const auto& path : paths)
                        if (carburettorTune(edited, path.id))
                            if (error = tuneCarburettor(edited, path.id, tuned); !error.empty()) return error;
                    return error;
                };
            }
        }
        subtitle = (exhaust ? "Exhaust path " : "Intake path ") + juce::String(duct.pathId);
        if (duct.kind == render::DuctKind::intakeRunner) subtitle = cylinderName;
        if (duct.authoredLengthMm > 1.0F) add("Length", millimetres(model_.units, duct.authoredLengthMm));
        if (exhaust) {
            for (const auto& path : config.exhaustPaths) {
                if (path.id != duct.pathId) continue;
                const auto network = path.network ? *path.network : makeEditableExhaustNetwork(path);
                for (const auto& component : network.components) {
                    if (component.id != duct.elementId || duct.kind != render::DuctKind::exhaustComponent) continue;
                    add("Diameter", millimetres(model_.units, component.diameterMm));
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
                // A difference to ambient: converted, never offset again.
                const auto& unitFormat = model_.units;
                auto difference = unitFormat;
                difference.preferences.pressureReference = units::PressureReference::absolute;
                const auto pressureUnit = difference.symbol(units::Quantity::pressure);
                const auto signedPressure = [&difference](float pa) {
                    return (pa >= 0.0F ? "+" : "")
                        + difference.value(units::Quantity::pressure, static_cast<double>(pa) / 1'000.0,
                                           difference.preferences.of(units::Quantity::pressure).decimals + 1);
                };
                add("Pressure vs ambient", element.sampleCount > 1
                        ? signedPressure(low) + " to " + signedPressure(high) + " " + pressureUnit
                        : signedPressure(low) + " " + pressureUnit);
                add("Gas temperature", unitFormat.text(units::Quantity::temperature,
                    static_cast<double>(gas / static_cast<float>(element.sampleCount)) - 273.15));
                if (wall > 0.0F)
                    add("Wall temperature", unitFormat.text(units::Quantity::temperature, static_cast<double>(wall) - 273.15));
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

std::optional<std::array<AcousticPoint3M, 2>> EngineViewport::cameraMicrophones() const {
    if (!settings_.cameraMicrophone || !scene_ || context_.getTargetComponent() == nullptr) return std::nullopt;
    // Where the camera is going: the sound glides there on its own.
    const auto orbit = goal();
    const auto eye = orbitEye(orbit.target, orbit.yaw, orbit.pitch, orbit.distance);
    const auto forward = render::normalise(orbit.target - eye);
    const auto right = render::normalise(render::cross(forward, { 0.0F, 1.0F, 0.0F }));
    return listenerFrame_.microphones(eye, right, microphoneSpacingM_);
}

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
    menu.addSectionHeader(utf8("Exhaust waves"));
    menu.addItem(30, utf8("Standard (360 mm cells; two-strokes 100 mm)"), true, !settings_.fineExhaustWaves);
    menu.addItem(31, utf8("Fine (180 mm cells, slower)"), true, settings_.fineExhaustWaves);
    menu.addSectionHeader(utf8("Pressure waves (All layer)"));
    menu.addItem(40, utf8("Hidden"), use3d, settings_.pressureWaves == ViewSettings::PressureWaves::hidden);
    menu.addItem(41, utf8("Pulsation strength (steady)"), use3d,
                 settings_.pressureWaves == ViewSettings::PressureWaves::strength);
    menu.addItem(42, utf8("Live pulses"), use3d, settings_.pressureWaves == ViewSettings::PressureWaves::live);
    menu.addSectionHeader(utf8("Sound"));
    menu.addItem(50, utf8("Microphone on the camera"), use3d, settings_.cameraMicrophone);
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
            } else if (result == 50) {
                settings.cameraMicrophone = !settings.cameraMicrophone;
            } else if (result >= 40 && result <= 42) {
                settings.pressureWaves = static_cast<ViewSettings::PressureWaves>(result - 40);
                safe->pressureScalePa_ = 0.0F;
            } else if ((result == 30 || result == 31) && settings.fineExhaustWaves != (result == 31)) {
                settings.fineExhaustWaves = result == 31;
                settings.save();
                if (safe->onExhaustResolutionChanged) safe->onExhaustResolutionChanged();
                return;
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
        // The keys and the wheel change the time scale too.
        auto speed = -1;
        for (std::size_t i = 0; i < simulationSpeeds.size(); ++i)
            if (std::abs(health.timeScale - simulationSpeeds[i]) < 1.0e-3) speed = static_cast<int>(i);
        speed_.setSelected(speed);
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
        shared_.pose.powerValveOpening = state.powerValveOpening;
        shared_.turboShaftRpm = state.forcedInductionShaftSpeedRpm;
        // In gear the input shaft runs at the clutch's driven side; in
        // neutral the engaged clutch spins it with the crank.
        shared_.pose.engagedGear = state.gear;
        shared_.gearboxInputRpm = state.gear == -1 ? state.rpm : state.rpm - state.clutchSlipRpm;
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
    const auto right = speed_.isVisible() ? speed_.getX() - 12 : getWidth() - 12;
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
    speed_.setBounds(playback_.getX() - 10 - speed_.idealWidth(), getHeight() - 44, speed_.idealWidth(), 32);
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
    const auto waves = effectivePressureWaves(shared.pressureWaves, shared.layer);
    gasFieldView_.setColouring(waves == ViewSettings::PressureWaves::live ? render::GasFieldView::Colouring::wave
                                                                          : render::GasFieldView::Colouring::strength);
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
    frame_.showWaves = waves != ViewSettings::PressureWaves::hidden;
    frame_.xray = shared.xray;
    frame_.antiAliasing = shared.antiAliasing;
    frame_.pose = shared.pose;
    frame_.pose.afterfire = gasFieldView_.afterfire();
    // The turbo shaft turns at its simulated speed, slowed down with the crank.
    const auto shaftDegreesPerSecond
        = 6.0 * shared.turboShaftRpm * (shared.playbackFactor >= 0.999 ? shared.rate : shared.playbackFactor);
    turboShaftDegrees_ = std::fmod(turboShaftDegrees_ + shaftDegreesPerSecond * dt, 360.0);
    frame_.pose.turboShaftDegrees = turboShaftDegrees_;
    const auto gearboxDegreesPerSecond
        = 6.0 * shared.gearboxInputRpm * (shared.playbackFactor >= 0.999 ? shared.rate : shared.playbackFactor);
    gearboxInputDegrees_ += gearboxDegreesPerSecond * dt;
    frame_.pose.gearboxInputDegrees = gearboxInputDegrees_;
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
    frame_.gearboxInputSweepDegrees = shared.motionBlur ? gearboxDegreesPerSecond * 0.5 * frameInterval_ : 0.0;

    renderer_->render(frame_, width, height);
    displayedAngle_.store(angle);
    framesRendered_.fetch_add(1);
}

void EngineViewport::openGLContextClosing() {
    if (renderer_) renderer_->release();
    renderer_.reset();
}

} // namespace enginelab::ui
