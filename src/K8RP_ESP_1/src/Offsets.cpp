// ====================================================================================================================
//             Offsets
//             Loads the current offsets from disk
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "Offsets.h"

#include <Preferences.h>
#include <nvs_flash.h>  // for wiping NVram

#include <iterator>

Preferences preferences;

#define PREF_NAME "K8RP"

namespace Naigon::BB_8 {

enum OffsetIndicies : uint8_t {
    Pitch = 0,
    Roll = 1,
    SideToSide = 2,
    DomeSpin = 3
};

Offsets::Offsets()
    // Pitch and Roll are float; remaining are int.
    : _offsetsNeedWrite(false), _domeOffsetNeedsWrite(false), _isValid(false) {
    for (uint8_t i = 0; i < 5; i++) {
        _offsetIsFloat[i] = i < 2 ? true : false;
    }
}

bool Offsets::NeedsWrite() const {
    return _offsetsNeedWrite || _domeOffsetNeedsWrite;
}

bool Offsets::AreValuesLoaded() const {
    // It can be valid from initial load, or by manually setting values for both dome and normal.
    return _isValid || (_isDomeValid && _areOffsetsValid);
}

float Offsets::PitchOffset() const {
    return _offsets[OffsetIndicies::Pitch];
}

float Offsets::RollOffset() const {
    return _offsets[OffsetIndicies::Roll];
}

int Offsets::SideToSidePotOffset() const {
    return _offsets[OffsetIndicies::SideToSide];
}

int Offsets::DomeSpinPotOffset() const {
    return _offsets[OffsetIndicies::DomeSpin];
}

float Offsets::PrintOffsets() const {
    return _offsets[OffsetIndicies::Pitch];
}

bool Offsets::LoadOffsetsFromMemory() {
    float sum = 0.0f;
    preferences.begin(PREF_NAME, false);

    _offsets[OffsetIndicies::Pitch] = preferences.getFloat("pitchOffset", 0.0f);
    _offsets[OffsetIndicies::Roll] = preferences.getFloat("rollOffset", 0.0f);
    _offsets[OffsetIndicies::SideToSide] = preferences.getInt("s2sOffset", 0);
    _offsets[OffsetIndicies::DomeSpin] = preferences.getInt("domeOffset", 0);

    preferences.end();

    for (uint8_t i = 0; i < 4; i++) {
        sum += _offsets[i];
    }

    // If the sum is non-zero(ish) then we can assume values were loaded. Zero means this is the first time the user
    // ran the drive and values need to be stored.
    _isValid = abs(sum) > 0.05f;
    return _isValid;
}

void Offsets::UpdateOffsets(float pitchOffset, float rollOffset, int sideToSideOffset, int domeSpinOffset) {
    _offsets[OffsetIndicies::Pitch] = pitchOffset * -1.0f;
    _offsets[OffsetIndicies::Roll] = rollOffset * -1.0f;
    _offsets[OffsetIndicies::SideToSide] = 0 - sideToSideOffset;
    _offsets[OffsetIndicies::DomeSpin] = 0 - domeSpinOffset;

    _offsetsNeedWrite = true;
    _areOffsetsValid = true;
    _writeStep = 0;
}

void Offsets::WriteOffsets() {
    preferences.begin(PREF_NAME, false);
    // No-op if nothing needs to be written.
    if (!NeedsWrite()) {
        return;
    }

    if (_offsetsNeedWrite) {
        preferences.putFloat("pitchOffset", _offsets[OffsetIndicies::Pitch]);
        preferences.putFloat("rollOffset", _offsets[OffsetIndicies::Roll]);
        preferences.putInt("s2sOffset", _offsets[OffsetIndicies::SideToSide]);
        preferences.putInt("domeOffset", _offsets[OffsetIndicies::DomeSpin]);

        _offsetsNeedWrite = false;
    }

    delay(500);
    preferences.end();
}

void Offsets::ClearOffsets() {
    preferences.begin(PREF_NAME, false);
    preferences.clear();
    delay(1000);
    preferences.end();

    _offsetsNeedWrite = true;
    _areOffsetsValid = true;
    _isValid = true;
    _writeStep = 0;
}

}  // namespace Naigon::BB_8
