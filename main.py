# main.py
import os
from dotenv import load_dotenv
import chainlit as cl
from chainlit.input_widget import Select
from model_fetcher import ModelFetcher
from chatbot import Chatbot

# Load environment variables from .env file (fallback for local development)
load_dotenv()

# ============================================================
# Initialisierung
# ============================================================
# Try to read from Docker secret first (more secure), then fall back to env var
secret_path = "/run/secrets/openrouter_api_key"
if os.path.exists(secret_path):
    with open(secret_path, 'r') as f:
        api_key = f.read().strip()
else:
    # Fallback to environment variable (for local development or if secret not used)
    api_key = os.getenv("OPENROUTER_API_KEY")

if not api_key:
    raise ValueError(
        "OPENROUTER_API_KEY nicht gefunden!\n"
        "Option 1 (Docker secret): Erstelle .env.secret mit: OPENROUTER_API_KEY=your_key\n"
        "Option 2 (env var): Setze OPENROUTER_API_KEY als Umgebungsvariable"
    )
fetcher = ModelFetcher()

# ============================================================
# Chat Start
# ============================================================
@cl.on_chat_start
async def on_chat_start():
    """Wird ausgeführt, wenn der Benutzer den Chat startet."""
    models = fetcher.get_models(free=True, tools=True)

    if not models:
        await cl.Message(
            content="Keine kostenlosen Modelle mit Tool-Unterstützung gefunden."
        ).send()
        return

    model_options = [m["id"] for m in models]

    # Benutzer soll Modell wählen
    settings = await cl.ChatSettings(
        [
            Select(
                id="Model",
                label="🔘 Wähle ein OpenRouter-Modell mit Tool-Unterstützung",
                values=model_options,
                initial_index=0,
            )
        ]
    ).send()

    chosen_model = settings["Model"]
    chatbot = Chatbot(api_key, chosen_model)

    # Session speichern
    cl.user_session.set("chatbot", chatbot)
    cl.user_session.set("model", chosen_model)
    cl.user_session.set("api_key", api_key)

    await cl.Message(
        content=f"Modell gesetzt auf **{chosen_model}**.\n\n Willkommen! Stelle mir deine erste Frage!"
    ).send()


# ============================================================
# Modell-Wechsel
# ============================================================
@cl.on_settings_update
async def on_settings_update(settings: dict):
    """Reagiert, wenn der Nutzer das Modell im UI ändert."""
    new_model = settings.get("Model")
    old_model = cl.user_session.get("model")
    chatbot: Chatbot = cl.user_session.get("chatbot")
    api_key = cl.user_session.get("api_key")

    if new_model != old_model:
        chatbot.update_model(api_key, new_model)
        cl.user_session.set("model", new_model)

        await cl.Message(
            content=f"Modell erfolgreich gewechselt!\n\nAktuelles Modell: **{new_model}**"
        ).send()


# ============================================================
# Nachrichtenverarbeitung
# ============================================================
@cl.on_message
async def on_message(message: cl.Message):
    """Behandelt eingehende Nachrichten."""
    chatbot: Chatbot = cl.user_session.get("chatbot")
    if not chatbot:
        await cl.Message(content="Kein Modell aktiv. Bitte starte den Chat neu.").send()
        return

    answer = await chatbot.get_response(message.content)

    # Send the response
    await cl.Message(content=answer).send()

