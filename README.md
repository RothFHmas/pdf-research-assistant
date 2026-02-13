# PDF Research Assistant

Ein Research‑Chatbot, der wissenschaftliche PDFs automatisch durchsucht, relevante Inhalte extrahiert und beim Literaturstudium unterstützt.

> 🧠 *Dieses Projekt ist eine Abwandlung des an der FH‑Technikum Wien im Rahmen des Kurses „Moderne Roboterkonzepte“ durchgeführten Projekts meiner Gruppe. ([github.com/DavidSeyserGit/Moderne_Roboterkonzepte](https://github.com/DavidSeyserGit/Moderne_Roboterkonzepte))*  

---

## Überblick

Der **PDF Research Assistant** ist ein KI‑basierter Chatbot, der:

- PDF‑Dokumente verarbeitet und analysiert  
- relevante Text‑Passagen extrahiert  
- Fragen zu wissenschaftlichen Arbeiten beantwortet  

Ziel ist es, Forschenden und Studierenden die Arbeit mit wissenschaftlicher Literatur zu erleichtern – z. B. beim Zusammenfassen, Verstehen und Recherchieren.

---

## Funktionen

✔️ Durchsucht mehrere PDF‑Dokumente automatisch  
✔️ Extrahiert semantisch relevante Inhalte  
✔️ Beantwortet Nutzeranfragen, basierend auf PDF‑Inhalten  
✔️ Unterstützt interaktives Chat‑basiertes Arbeiten  
✔️ Ideal für Literaturstudium, Review‑Prozesse und Forschungsvorbereitung

---

## Installation

1. Repository klonen:
   ```bash
   git clone https://github.com/RothFHmas/pdf-research-assistant.git
   cd pdf-research-assistant
   ```
2. Anaconda‑Environment erstellen und aktevieren (falls noch nicht vorhanden):
```bash
conda create -n chatbot python=3.10
conda activate chatbot
```
3. Abhängigkeiten installieren:
```bash
pip install -r requirements.txt
```

---

## Nutzung
1. PDFs in das vorgesehen Verzeichnis legen (```./data/pdfs```)
2. Eine .env Datei erstellen / aktualisieren und den OpenRouter ([https://openrouter.ai](https://openrouter.ai)) API-Key eintragen:
```bash
OPENROUTER_API_KEY=dein_api_key_hier
```
Wenn man nur Gratis modelle verwenden will sollte der API key mit einem credit-limit von 0 erstellt werden!      
3. Starten mit Chainlit:
```bash
conda activate chatbot
chainlit run ./src/main.py
```
3. Chainlit öffnet das Interface im Browser (normalerweise unter ```http://localhost:8000```)
4. Fragen zu den Dokumenten im Chat‑Interface stellen

Bei Bedarf kann die Systemprompt unter ```./config/System_prompt.txt``` angepasst werden.   
Der Chatbot besitzt auch winige wietere tools und kann als "normaler" chatbot eingesetzt werden.




















