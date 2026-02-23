#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

namespace
{
const char *kApSsid = "BalanceBot-Config";
const char *kApPassword = nullptr; // open AP
const char *kPreferenceNamespace = "app";
const char *kPreferenceKey = "text_pref";

Preferences preferences;
WebServer server(80);

String escapeHtmlAttribute(const String &input)
{
  String escaped;
  escaped.reserve(input.length());

  for (size_t i = 0; i < input.length(); ++i)
  {
    const char c = input.charAt(i);
    switch (c)
    {
    case '&':
      escaped += "&amp;";
      break;
    case '"':
      escaped += "&quot;";
      break;
    case '<':
      escaped += "&lt;";
      break;
    case '>':
      escaped += "&gt;";
      break;
    default:
      escaped += c;
      break;
    }
  }

  return escaped;
}

String getConfigPage()
{
  const String savedValue = escapeHtmlAttribute(preferences.getString(kPreferenceKey, ""));

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Config</title>
</head>
<body>
  <h2>Device Config</h2>
  <form method="POST" action="/save">
    <label for="pref">Preference</label><br>
    <input type="text" id="pref" name="pref" value=")rawliteral";
  html += savedValue;
  html += R"rawliteral("><br><br>
    <button type="submit">Save</button>
  </form>
</body>
</html>
)rawliteral";

  return html;
}

void setUpWebServer()
{
  server.on("/", HTTP_GET, []()
            { server.send(200, "text/html", getConfigPage()); });

  server.on("/save", HTTP_POST, []()
            {
    if (!server.hasArg("pref"))
    {
      server.send(400, "text/plain", "Missing field: pref");
      return;
    }

    const String value = server.arg("pref");
    preferences.putString(kPreferenceKey, value);

    server.send(200, "text/plain", "Saved."); });
}

void startAccessPoint()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  Serial.print("AP SSID: ");
  Serial.println(kApSsid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}
} // namespace

void setup()
{
  Serial.begin(115200);
  delay(250);

  preferences.begin(kPreferenceNamespace, false);

  startAccessPoint();
  setUpWebServer();
  server.begin();
}

void loop()
{
  server.handleClient();
  delay(10);
}
