# model fetcher.py

import requests

class ModelFetcher:
    #Durchsucht alle verfügbaren Modelle nach den gewünschten Funktionen.
    API_URL = "https://openrouter.ai/api/v1/models"

    def __init__(self):
        self.price_fields = [
            "prompt", "completion", "request",
            "image", "web_search", "internal_reasoning",
            "input_cache_read", "input_cache_write"
        ]

    def get_models(
        self,
    ):
        response = requests.get(self.API_URL)
        if response.status_code != 200:
            raise RuntimeError(f"API request failed: {response.status_code}")

        all_models = response.json().get("data", [])
 
        return all_models