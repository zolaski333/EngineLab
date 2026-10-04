#include <enginelab/app/EngineViewport.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
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

    addChildComponent(cutaway_);
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

void EngineViewport::setEngine(const EngineConfig& config) {
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
        if (scene_) shared_.goal = presetOrbit(view_);
        shared_.smoothingSeconds = 0.0F;
    }
    resized();
    cutaway_.repaint();
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
    // Front looks at the crank nose (the flywheel is at +Z).
    static constexpr std::array<render::Vec3, 3> directions { render::Vec3 { -0.02F, 0.18F, -1.0F },
                                                              render::Vec3 { 1.0F, 0.18F, 0.02F },
                                                              render::Vec3 { 1.05F, 0.55F, 0.85F } };
    const auto direction = render::normalise(directions[static_cast<std::size_t>(std::clamp(view, 0, 2))]);
    const auto& bounds = scene_->bounds();
    Orbit orbit;
    orbit.target = bounds.centre();
    orbit.pitch = std::asin(direction.y);
    orbit.yaw = std::atan2(direction.x, direction.z);
    orbit.distance = 0.5F * bounds.diagonal() / std::sin(0.5F * fieldOfViewRadians) * 0.9F;
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
    view_ = std::clamp(view, 0, 2);
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
    dragStartOrbit_ = goal();
    panning_ = event.mods.isRightButtonDown() || event.mods.isMiddleButtonDown();
}

void EngineViewport::mouseDrag(const juce::MouseEvent& event) {
    if (!scene_) return;
    const auto delta = event.position - dragStart_;
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

void EngineViewport::mouseUp(const juce::MouseEvent&) { panning_ = false; }

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
    if (use3d) {
        const auto& state = model_.state;
        const auto& health = model_.health;
        const auto firing = (state.ecuSparkEnabled || model_.config.fuel == FuelType::diesel) && state.ecuFuelEnabled
            && !state.ecuDecelerationFuelCutActive && state.rpm > 60.0;
        const std::lock_guard lock(sharedMutex_);
        shared_.sampleWall = now;
        shared_.crankAngle = state.crankAngleDegrees;
        shared_.rpm = state.rpm;
        // Simulated seconds per wall second: the time scale, slowed further
        // when the physics cannot keep up.
        shared_.rate = health.paused ? 0.0 : health.timeScale * std::clamp(health.realtimeFactor, 0.0, 1.0);
        shared_.sampleSequence = ++sampleSequence_;
        shared_.pose.rpm = state.rpm;
        shared_.pose.throttle = state.throttle;
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
    return { left, getHeight() - 30, std::max(0, right - left), 18 };
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
    const auto diagonal = scene->bounds().diagonal();
    frame_.eye = eye;
    frame_.view = render::Mat4::lookAt(eye, orbit_.target, { 0.0F, 1.0F, 0.0F });
    frame_.projection = render::Mat4::perspective(fieldOfViewRadians,
        static_cast<float>(width) / static_cast<float>(height),
        std::max(0.5F, 0.01F * orbit_.distance), orbit_.distance + 4.0F * diagonal);
    frame_.layer = shared.layer;
    frame_.xray = shared.xray;
    frame_.antiAliasing = shared.antiAliasing;
    frame_.pose = shared.pose;

    // Motion blur: the shutter stays open for half a frame, and the crank is
    // drawn every 12 degrees it sweeps in that time (12 sub-frames at most).
    // Every 6 degrees, up to 24, took 2.5 % of the physics capacity on a
    // GTX 1660 Super (docs/journal.md).
    const auto sweep = shared.motionBlur ? clock_.displayDegreesPerSecond() * 0.5 * frameInterval_ : 0.0;
    const auto subframes = std::clamp(static_cast<int>(std::ceil(sweep / 12.0)), 1, 12);
    frame_.angles.resize(static_cast<std::size_t>(subframes));
    for (int k = 0; k < subframes; ++k)
        frame_.angles[static_cast<std::size_t>(k)] = angle - sweep * static_cast<double>(subframes - 1 - k) / subframes;

    renderer_->render(frame_, width, height);
    displayedAngle_.store(angle);
    framesRendered_.fetch_add(1);
}

void EngineViewport::openGLContextClosing() {
    if (renderer_) renderer_->release();
    renderer_.reset();
}

} // namespace enginelab::ui
