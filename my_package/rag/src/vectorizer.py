from sklearn.feature_extraction.text import TfidfVectorizer
import MeCab
import logging

# OpenAIのHTTPリクエストログを無効化
logging.getLogger("httpx").setLevel(logging.WARNING)


class JapaneseTokenizer:
    def __init__(self):
        self.tagger = MeCab.Tagger("-Owakati")

    def __call__(self, text):
        return self.tagger.parse(text).split()


class Vectorizer:
    def __init__(self, client):
        self.client = client
        self.tokenizer = JapaneseTokenizer()
        self.vectorizer = TfidfVectorizer(tokenizer=self.tokenizer, lowercase=False)

    async def get_embedding(self, text):
        response = await self.client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding

    def fit_transform(self, texts):
        return self.vectorizer.fit_transform(texts)

    def transform(self, texts):
        return self.vectorizer.transform(texts)

    def get_feature_names(self):
        return self.vectorizer.get_feature_names_out()
