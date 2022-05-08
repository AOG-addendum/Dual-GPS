#if HardwarePlatform == 0
//---------------------------------------------------------------------
// start WiFi in Workstation mode = log to existing WiFi

// WIFI handling 15. Maerz 2020 for ESP32  -------------------------------------------

void WiFi_Start_STA_Async() {
  static unsigned long timeout = millis();
  static bool wifiToggle = false;
  static bool wifiConfigured = false;
  if (millis() - timeout > 2000 and WiFi.status() != WL_CONNECTED) {
    timeout = millis();
    if (wifiToggle == true) {
      WiFi.disconnect(true);
    }
    else {
      Start_WiFi();
      Serial.print("try to connect to WiFi asynchronously: "); Serial.println(GPSSet.ssid);
    }
    wifiToggle = not wifiToggle;
    wifiConfigured = false;
  }
  if (WiFi.status() == WL_CONNECTED and wifiConfigured == false) {
    Configure_WiFi();
    wifiConfigured = true;
  }
}

void Start_WiFi (){
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("dual_gps_module");
  WiFi.begin(GPSSet.ssid, GPSSet.password);  
}

void WiFi_Start_STA_Sync() {
  unsigned long timeout, timeout2, print_timeout;
  WiFi.mode(WIFI_STA);   //  Workstation
  Serial.print("try to connect to WiFi synchronously for ");
  Serial.print(GPSSet.timeoutRouter);
  Serial.print(" seconds: ");
  Serial.println(GPSSet.ssid);
  Start_WiFi();
  timeout = millis() + (GPSSet.timeoutRouter * 1000);
  timeout2 = timeout - (GPSSet.timeoutRouter * 500);
  print_timeout = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    if (millis() - print_timeout > 5000) {
      print_timeout = millis();
      Serial.print(".");
    }
    delay(5);
    if ((millis() > timeout2) && (WiFi.status() != WL_CONNECTED)) {
      WiFi.disconnect(true);
      delay(200);
      Start_WiFi();
      timeout2 = timeout + 100;
    }
    //WIFI LED blink in double time while connecting
    if (!LED_WIFI_ON) {
      if (millis() > (LED_WIFI_time + (LED_WIFI_pause >> 2)))
      {
        LED_WIFI_time = millis();
        LED_WIFI_ON = true;
        digitalWrite(GPSSet.LEDWiFi_PIN, !GPSSet.LEDWiFi_ON_Level);
      }
    }
    if (LED_WIFI_ON) {
      if (millis() > (LED_WIFI_time + (LED_WIFI_pulse >> 2))) {
        LED_WIFI_time = millis();
        LED_WIFI_ON = false;
        digitalWrite(GPSSet.LEDWiFi_PIN, GPSSet.LEDWiFi_ON_Level);
      }
    }
  }  //connected or timeout

  Serial.println(""); //NL
  if (WiFi.status() == WL_CONNECTED) {
    Configure_WiFi();
  }
  else {
    // WiFi.end();
    Serial.println("WLAN-Client-Connection failed");
    Serial.println();
    LED_WIFI_ON = false;
    digitalWrite(GPSSet.LEDWiFi_PIN, !GPSSet.LEDWiFi_ON_Level);
  }
  delay(20);
}

void Configure_WiFi () {
  delay(200);
  Serial.println();
  Serial.print("WiFi Client successfully connected to : ");
  Serial.println(GPSSet.ssid);
  Serial.print("Connected IP - Address : ");
  IPAddress myIP = WiFi.localIP();
  Serial.println(myIP);
  IPAddress gwIP = WiFi.gatewayIP();
  //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
  myIP[3] = GPSSet.myIPEnding; //set ESP32 IP to x.x.x.myIP_ending
  Serial.print("changing IP to: ");
  Serial.println(myIP);
  if (!WiFi.config(myIP, gwIP, GPSSet.mask, gwIP)) {
    Serial.println("STA Failed to configure");
  }
  delay(200);
  Serial.print("Connected IP - Address : ");
  myIP = WiFi.localIP();
  ipDestination = myIP;
  ipDestination[3] = 255;
  Serial.println(myIP);
  Serial.print("Gateway IP - Address : ");
  Serial.println(gwIP);
  GPSSet.ipDestination[0] = myIP[0];
  GPSSet.ipDestination[1] = myIP[1];
  GPSSet.ipDestination[2] = myIP[2];
  GPSSet.ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
  LED_WIFI_ON = true;
  digitalWrite(GPSSet.LEDWiFi_PIN, GPSSet.LEDWiFi_ON_Level);
  my_WiFi_Mode = 1;// WIFI_STA;

}

//---------------------------------------------------------------------
// start WiFi Access Point = only if no existing WiFi

void WiFi_Start_AP() {
  WiFi.mode(WIFI_MODE_APSTA);   // Accesspoint and station
  String apName = String( GPSSet.ssid_ap );
  apName += String( " " );
  apName += WiFi.macAddress();
  apName.replace( ":", "" );  
  WiFi.softAP(apName.c_str());
  while (!SYSTEM_EVENT_AP_START) // wait until AP has started
  {
    delay(100);
    Serial.print(".");
  }
  delay(100);//right IP adress only with this delay
  WiFi.softAPConfig(GPSSet.gwip, GPSSet.gwip, GPSSet.mask);  // set fix IP for AP
  delay(300);
  IPAddress myIP = WiFi.softAPIP();
  delay(300);

  //AP_time = millis();
  Serial.print("Accesspoint started - Name : ");
  Serial.println(apName);
  Serial.print( " IP address: ");
  ipDestination = myIP;
  Serial.println(ipDestination);
  ipDestination[3] = 255;
  LED_WIFI_ON = true;
  digitalWrite(GPSSet.LEDWiFi_PIN, GPSSet.LEDWiFi_ON_Level);
  my_WiFi_Mode = WIFI_AP;
}

//-------------------------------------------------------------------------------------------------
/* async only needs one definition .onPacket in setup so code put in setup and time check in main
void doUDPNtrip() {
	udpNtrip.onPacket([](AsyncUDPPacket packet)
		{
			if (GPSSet.debugmode) { Serial.println("got NTRIP data"); }
			for (unsigned int i = 0; i < packet.length(); i++)
			{
				Serial1.write(packet.data()[i]);
			}
			NtripDataTime = millis();
		});  // end of onPacket call
	if ((NtripDataTime + 30000) < millis())
	{
		NtripDataTime = millis();
		Serial.println("no NTRIP from AOG for more than 30s");
	}

}

*/
//-------------------------------------------------------------------------------------------------






















#endif

//-------------------------------------------------------------------------------------------------
/* from autosteer code, not for async udp
void UDP_Start()
{
    if (UDPToAOG.begin(steerSet.portMy))
    {
        Serial.print("UDP sendig to IP: ");
        for (byte n = 0; n < 4; n++) {
            Serial.print(steerSet.ipDestination[n]);
            Serial.print(".");
        }
        Serial.print(" from port: ");
        Serial.print(steerSet.portMy);
        Serial.print(" to port: ");
        Serial.println(steerSet.portDestination);
    }
    delay(300);
    if (UDPFromAOG.begin(steerSet.portAOG))
    {
        Serial.print("UDP listening for AOG data on IP: ");
        Serial.println(WiFi.localIP());
        Serial.print(" on port: ");
        Serial.println(steerSet.portAOG);
        getDataFromAOG();
    }
}*/
