#pragma once

// Called after SD.begin(), before controller/filter initialization.
void initializeParameterService(bool sdReady);
// Bounded serial RX/TX processing; call once at the end of a control cycle.
// SD writes are synchronous.
void pollParameterService();
