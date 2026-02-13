# tools.py
from langchain_core.tools import tool
from ddgs import DDGS
from rag import retrieve_context, index_all_pdfs, auto_index_if_needed

# Auto-index PDFs if needed on module import
try:
    status = auto_index_if_needed()
    print(f"[RAG] {status}")
except Exception as e:
    print(f"[RAG] Auto-index check failed: {e}")

@tool 
def get_current_date() -> str:
    """
    Docstring for get_current_date
    
    :return: The current date in YYYY-MM-DD format
    :rtype: str
    """
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d")

@tool
def hello_world() -> str:
    """
    Docstring for hello_world

    :return: A simple greeting message
    :rtype: str
    """
    return "Hello World!"

@tool
def get_current_time() -> str:
    """
    Docstring for get_current_time
    
    :return: The current time in HH:MM:SS format
    :rtype: str
    """
    from datetime import datetime
    return datetime.now().strftime("%H:%M:%S")

@tool
def get_current_datetime() -> str:
    """
    Docstring for get_current_datetime
    
    :return: The current date and time in YYYY-MM-DD HH:MM:SS format
    :rtype: str
    """
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

@tool
def echo(input_text: str) -> str:
    """
    Docstring for echo
    
    :param input_text: The text to echo back
    :type input_text: str
    :return: The same text that was input
    :rtype: str
    """
    return input_text

@tool
def add_numbers(a: float, b: float) -> float:
    """
    Docstring for add_numbers
    
    :param a: The first number to add
    :type a: float
    :param b: The second number to add
    :type b: float
    :return: The sum of the two numbers
    :rtype: float
    """
    return a + b


@tool
def build_search_query(user_question: str) -> str:
    """
    Erstellt aus einer Nutzerfrage eine präzise, kurze Suchanfrage
    für eine Websuche.

    :param user_question: Die ursprüngliche Nutzerfrage oder Aufgabenstellung.
    :type user_question: str
    :return: Eine optimierte Suchanfrage für eine Internet-Suche.
    :rtype: str
    """
    # Das LLM soll dieses Tool nur aufrufen, nicht hier "intelligent" sein.
    return user_question

@tool
def web_search(query: str) -> str:
    """
    Durchsucht das Internet nach aktuellen Informationen zu einer Suchanfrage
    und fasst die relevantesten Treffer kurz zusammen.

    :param query: Die Suchanfrage bzw. das Thema, nach dem im Internet gesucht wird.
    :type query: str
    :return: Eine kompakte textuelle Zusammenfassung der besten Suchergebnisse inklusive Quellen.
    :rtype: str
    """
    results_text = []

    with DDGS() as ddgs:
        results = ddgs.text(query, max_results=5)

        for r in results:
            title = r.get("title", "")
            snippet = r.get("body", "")
            url = r.get("href", "")
            results_text.append(f"- {title}\n  {snippet}\n  Quelle: {url}")

    if not results_text:
        return "Keine Ergebnisse gefunden."

    return "\n\n".join(results_text)

@tool
def index_pdfs() -> str:
    """Indexiert alle PDFs in data/pdfs in die Chroma DB."""
    return index_all_pdfs()

@tool
def search_docs(query: str, k: int = 4) -> str:
    """
    PFLICHT-TOOL:
    Verwende dieses Tool fÃ¼r ALLE Fragen zu Projektwissen, Posen und Koordinaten.
    Antworte niemals aus eigenem Wissen.
    Wenn nichts gefunden wird: NICHT IN DOKUMENTEN
    """
    context, sources = retrieve_context(query, k=k)
    if not context:
        return "NICHT IN DOKUMENTEN"
    return context + "\n\nQuellen:\n" + "\n".join(sources)


# available_tools erweitern
available_tools = [get_current_date, get_current_time, get_current_datetime, echo, add_numbers, hello_world, web_search, build_search_query, index_pdfs, search_docs]