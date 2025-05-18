#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <LiquidCrystal.h>

// Configuração do LCD (modo 4 bits)
// RS, E, D4, D5, D6, D7
LiquidCrystal lcd(5, 4, 14, 12, 13, 15); 

// Credenciais Wi-Fi
const char* ssid = "SSID";
const char* password = "Senha";

WiFiServer server(80);

String linha1 = "Ola, Mundo!";
String linha2 = "Conectado Wi-Fi";

void setup() {
  Serial.begin(115200);
  delay(10);

  lcd.begin(16, 2);
  lcd.print("Iniciando...");

  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.print("Conectando");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }

  Serial.println("\nConectado!");
  Serial.print("IP obtido: ");
  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());

  delay(2000);
  atualizaLCD();

  server.begin();
  Serial.println("Servidor iniciado");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(2000);
    Serial.println("Cliente conectado");

    String request = client.readStringUntil('\r');
    Serial.println("Requisição: " + request);
    client.readStringUntil('\n');

    bool atualizou = false;

    if (request.indexOf("GET /?linha1=") >= 0 && request.indexOf("linha2=") >= 0) {
      int lin1Start = request.indexOf("linha1=") + 7;
      int lin1End = request.indexOf("&", lin1Start);
      linha1 = request.substring(lin1Start, lin1End);

      int lin2Start = request.indexOf("linha2=") + 7;
      int lin2End = request.indexOf(" HTTP", lin2Start);
      linha2 = request.substring(lin2Start, lin2End);

      linha1 = urlDecode(linha1);
      linha2 = urlDecode(linha2);

      atualizaLCD();
      atualizou = true;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    client.println("<!DOCTYPE html><html>");
    client.println("<head>");
    client.println("<meta charset='UTF-8'>");
    client.println("<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>");
    client.println("<meta http-equiv='Pragma' content='no-cache'>");
    client.println("<meta http-equiv='Expires' content='0'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
    client.println("<title>LCD Control</title>");
    if (atualizou) {
      client.println("<meta http-equiv='refresh' content='2;url=/' />");
    }
    client.println("</head>");
    client.println("<body>");
    if (atualizou) {
      client.println("<h3>Mensagem enviada com sucesso!</h3>");
    }
    client.println("<h2>Enviar Mensagem para o LCD</h2>");
    client.print("<p><b>Linha 1:</b> " + linha1 + "<br>");
    client.print("<b>Linha 2:</b> " + linha2 + "</p>");
    client.println("<form action=\"/\" method=\"GET\">");
    client.print("Linha 1: <input type=\"text\" name=\"linha1\" maxlength=\"16\" value=\"" + linha1 + "\"><br>");
    client.print("Linha 2: <input type=\"text\" name=\"linha2\" maxlength=\"16\" value=\"" + linha2 + "\"><br>");
    client.println("<input type=\"submit\" value=\"Enviar\"></form>");
    client.println("</body></html>");
    
    client.stop();
    delay(1);
  }
}

void atualizaLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(truncateTo16(linha1));
  lcd.setCursor(0, 1);
  lcd.print(truncateTo16(linha2));
}

String truncateTo16(String str) {
  return str.length() > 16 ? str.substring(0, 16) : str;
}

String urlDecode(String input) {
  String decoded = "";
  char temp[] = "0x00";
  unsigned int len = input.length();
  unsigned int i = 0;

  while (i < len) {
    char c = input.charAt(i);
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && i + 2 < len) {
      temp[2] = input.charAt(i + 1);
      temp[3] = input.charAt(i + 2);
      decoded += (char) strtol(temp, NULL, 16);
      i += 2;
    } else {
      decoded += c;
    }
    i++;
  }

  return decoded;
}
