# Moderne_Roboterkonzepte

## GUI
Für das GUI selbst wird Chainlit verwendet Hiermeit bekommen wir eine schöne, Chat GPT ähnliche Chatoberfläche im Browser
- https://docs.chainlit.io/get-started/overview
### model_fetcher
Der Model fetcher ruft die API von openrouter auf und extrahiert von dort alle verfügmaren modelle als .json  
Zeile 37 `response = requests.get(self.API_URL)`  
Zeile 41 `models = response.json().get("data", [])`  
Diese json wir nun nach bestimmten parametern die die einzelnen modelle unterstüzen durchsucht.  

Es wird einerseits nach dem priceing gefilterd (in unserem fall welche modelle gratis sind (diese funktionalität kann ausgeschaltet werden)), und nach allen support_parametern z.b. die unterstüzung von tools.  
Alle support parameter findet man hier: https://openrouter.ai/docs/overview/models alle implementierten in `model_fetcher` zeilen 14 bis 24.

Wenn alle gewünschten funktionen im modell vorhanden sind werden diese in die filtered_models liste aufgenommen.
Die filtered_models liste wird dann ausgegeben sobald alle modelle durchsucht wurden.
### chatbot
