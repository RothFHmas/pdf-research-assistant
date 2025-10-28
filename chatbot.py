from langchain_openai import ChatOpenAI
from langchain.chains import ConversationChain
from langchain.memory import ConversationBufferMemory

class Chatbot:
    """Chatbot-Klasse mit LangChain-Speicher und Modellwechsel."""
    def __init__(self, api_key: str, model: str):
        self.model_name = model
        self.llm = ChatOpenAI(
            openai_api_base="https://openrouter.ai/api/v1",
            openai_api_key=api_key,
            model=model,
        )
        self.memory = ConversationBufferMemory(return_messages=True)
        self.chain = ConversationChain(
            llm=self.llm,
            memory=self.memory,
            verbose=False
        )

    def update_model(self, api_key: str, model: str):
        """Wechselt das aktuelle Modell dynamisch."""
        self.model_name = model
        self.llm = ChatOpenAI(
            openai_api_base="https://openrouter.ai/api/v1",
            openai_api_key=api_key,
            model=model,
        )
        self.chain.llm = self.llm

    def get_response(self, user_input: str) -> str:
        """Verarbeitet Nutzereingaben mit Memory."""
        response = self.chain.run(input=user_input)
        return response.strip()
