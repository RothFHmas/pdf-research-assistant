# PDF Research Assistant

Ein Research‑Chatbot, der wissenschaftliche PDFs automatisch durchsucht, relevante Inhalte extrahiert und beim Literaturstudium unterstützt.

> 🧠 *Dieses Projekt ist eine Abwandlung des an der FH‑Technikum Wien im Rahmen des Kurses „Moderne Roboterkonzepte“ durchgeführten Projekts meiner Gruppe. ([Projekt Moderne Roboterkonzepte](https://github.com/DavidSeyserGit/Moderne_Roboterkonzepte))*  

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
conda create chatbot python=3.11
conda activate chatbot
 ```
3. Abhängigkeiten installieren:
```bash
pip install -r requirements.txt
```

---

## Nutzung
1. PDFs in das vorgesehen Verzeichnis legen (```./data/pdfs```)
2. Erstelle eine Datei namens `.env` **im Hauptverzeichnis (root) des Projekts** und trage darin deinen [OpenRouter](https://openrouter.ai/settings/keys) API-Key ein:  
```text
OPENROUTER_API_KEY=dein_api_key_hier
```
Wenn man nur Gratis modelle verwenden will sollte der API key mit einem credit-limit von 0 erstellt werden!      
3. Starten mit Chainlit:
### Option A: Direkt über die Konsole (klassisch)
```bash
conda activate chatbot
chainlit run ./src/main.py
```
### Option B: Über den Launcher (empfohlenes, schöneres Interface)

Alternativ kann der Chatbot auch über das grafische Launcher-Fenster gestartet werden.
Dieses bietet ein komfortableres Interface zum:
- Eintragen und Speichern des OpenRouter API-Keys
- Öffnen des PDF-Ordners zum Hinzufügen neuer Dokumente
- Starten/Beenden des Chatbots
- Anzeigen der Konsolen-Ausgabe
Start des Launchers:
```bash
conda activate chatbot
python launcher.py
```
3. Chainlit öffnet das Interface im Browser (normalerweise unter ```http://localhost:8000```)
4. Fragen zu den Dokumenten im Chat‑Interface stellen

### Nach erstmaligem Probieren

Die Datei `Start_Chatbot.bat` ist eine Batch-Datei, die in der Windows-CMD das Conda-Environment `chatbot` aktiviert und anschließend die `launcher.py` startet.  

Sie wurde erstellt, um den alltäglichen Gebrauch zu vereinfachen, sodass der Chatbot bequem ohne direkte Interaktion mit der Konsole gestartet werden kann.

### Starten per Verknüpfung (Windows)

Um den Chatbot besonders bequem zu starten, kann eine **Verknüpfung** zu der Batch-Datei `Start_Chatbot.bat` erstellt werden:

1. Navigiere zu dem Ordner, in dem `Start_Chatbot.bat` liegt.
2. Rechtsklick auf die Datei → **Senden an → Desktop (Verknüpfung erstellen)**  
   Oder: Rechtsklick → **Verknüpfung erstellen** und die Verknüpfung an gewünschter Stelle ablegen.

Ab jetzt reicht ein **Doppelklick auf die Verknüpfung**, um das Conda-Environment `chatbot` zu aktivieren und den Launcher zu starten – **ohne manuelles Öffnen der CMD**.

## Zusatz-Infos
Falls die Ergebnisse nicht zufriedenstellend sind, empfiehlt es sich, im Optionsmenü ⚙️ das verwendete LLM-Modell zu ändern.

   
Als Beispiel ist in ```./data/pdfs``` das paper zu [Open3D](https://arxiv.org/abs/1801.09847).   

Bei Bedarf kann die Systemprompt unter ```./config/System_prompt.txt``` angepasst werden.   

Der Chatbot besitzt auch winige wietere tools und kann als "normaler" chatbot eingesetzt werden.   




























