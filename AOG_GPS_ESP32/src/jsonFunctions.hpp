

#include "main.hpp"


// ESP32 pollutes the env with macros for all possible binary values starting with "B" (pe "B1 = 1", "B1111=7"...)
// undef them, as json.hpp defines a template argument with B1
#undef B1
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#pragma once

extern void loadSavedConfig();
extern void saveConfig();

extern json loadJsonFromFile( const char* fileName );
extern void saveJsonToFile( const json& json, const char* fileName );

extern void parseJsonToGpsConfig( json& json, GPS_Config& config );
extern json parseGpsConfigToJson( const GPS_Config& config );

extern void parseJsonToDiagnostics( json& json, Diagnostics& diagnostics );
extern json parseDiagnosticsToJson( const Diagnostics& diagnostics );
