# Rasa model

To understand how this model was used in a very similar application, see the [following code from DRAGON project](https://github.com/Shuijing725/Dragon_Wayfinding/blob/main/main.py#L408). The intents and entities were parsed directly from the response, and then the code was written to confirm or deny their objective. The problem here is that you need to manually program all the possible responses that come between a user interacting with the robot and finally actualizing that command. In order to completely utilize the model, we should include stories and rules, so that our chat bot can dynamically respond to the user and clarify. To see examples of this kind of training data, look [at this documentation](https://rasa.com/docs/rasa/training-data-format#example). 

For more documentation about the the `config.yml` that is provided by rasa, see the following [page](https://rasa.com/docs/rasa/model-configuration/). We are using the default configuration, it has just been un-commented for ease of use. Below I will go over each of the components that are included in the NLU pipeline:

[Whitespace tokenizer](https://rasa.com/docs/rasa/components#whitespacetokenizer)
- Tokenizer that separates based on whitespace 
- Need to indicate the token to split the intent on. In our case, `+`

[Regex featurizer](https://rasa.com/docs/rasa/components#regexfeaturizer)
- When training the regex featurizer, a list of regex patterns are extracted from the training data
- These regex features are then passed as additional features to the entity extractor and intent classifier
- It works on tokens as well as text. I believe the principle of this is similar to [this paper here](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a4d33fb21fae2552915e39885dca0b70904c1594).

[CountVectorsFeaturizer](https://rasa.com/docs/rasa/components/#countvectorsfeaturizer)
- Bag-of-words representation of user message and intent 
- This is based on [sklearn countvectorizer](https://scikit-learn.org/stable/modules/generated/sklearn.feature_extraction.text.CountVectorizer.html)
- Converts a collection of text into a matrix of token counts
- This is set up to default to n-grams of 1-4 characters (characters, not words). For a basic idea of what n-grams are, see [this stack overflow](https://stackoverflow.com/questions/18658106/quick-implementation-of-character-n-grams-for-word)

[DIETClassifier](https://rasa.com/docs/rasa/components/#dietclassifier)
- Our intent classifier / entity extractor
- You can change the size of this to change the size of the model
- For more information about this model, see [the publication](https://arxiv.org/abs/2004.09936)

[EntitySynonymMapper](https://rasa.com/docs/rasa/components/#entitysynonymmapper)
- This maps different spelling of the same word/entity to the same value
- Example: `NYC` and `New York City` both mapping to value of `nyc` for entity `city`

[ResponseSelector](https://rasa.com/docs/rasa/components/#responseselector)
- This is used for chit-chat, in which the model can be used to predict what will be said next
- We won't be using this, as we want rules to govern everything that will be said (at least at the start)

[FallbackClassifier](https://rasa.com/docs/rasa/components/#fallbackclassifier)
- Classifier that determines whether the real intent classifier will be used, or if nlu_fallback will be chosen
- If nlu_fallback is chosen, we can then write an action that tells the user what all the model is capable of doing


### Policies

Policies are way of dealing with messages after inference. It is the way the state of the dialogue is handled.

[MemoizationPolicy](https://rasa.com/docs/rasa/policies/#memoization-policy)
- Will use matches from stories to determine what to do next
- Will take the last max_history number of turns into account

[TEDPolicy](https://rasa.com/docs/rasa/policies/#ted-policy)
- 

NOTE that for the multi-intent classification, you must give explicit examples. From the documents: _The model will not predict any combination of intents for which examples are not explicitly given in training data. As accounting for every possible intent combination would result in combinatorial explosion of the number of intents, you should only add those combinations of intents for which you see enough examples coming in from real users._ 

To run, do `rasa run --enable-api` and to call, run:

```
curl http://0.0.0.0:5005/model/parse -s -d '{ "text": "Good morning, robot. I need you to go to room 205A and bring me the patients file" }' | python -m json.tool
```