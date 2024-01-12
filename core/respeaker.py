import rospy
import spacy # for nlp
from loguru import logger
import os
from mr_voice.msg import Voice
from std_msgs.msg import String
from typing import NamedTuple, List

class NLP_result(NamedTuple):
    id: str
    sentence: str
    matched_sentence: str
    similarity: float

class ReSpeaker_event(NamedTuple):
    nlp: List[NLP_result]
    time: int
    direction: int
    text: str

class NLP:
    def __init__(self, model_name="en_core_web_sm"):
        self.sentences = dict()
        self.model = spacy.load(model_name)
    def remove_sentence(self, sent_id) -> bool:
        if sent_id in self.sentences:
            del self.sentences[sent_id]
            return True
        else:
            return False

    def add_sentence(self, new_sentence) -> str:
        sent_id = os.urandom(16).hex()
        self.sentences[sent_id] = new_sentence
        return sent_id

    def check_sentence(self, sentence, topk=3) -> List[NLP_result]:
        results = []

        for sent_id, sent in self.sentences.items():
            doc1 = self.model(sentence)
            doc2 = self.model(sent)
            similarity = doc1.similarity(doc2)

            result = NLP_result(sentence=sentence,
                                matched_sentence=sent,
                                id=sent_id,
                                similarity=similarity)

            results.append(result)

        results.sort(key=lambda x: x.similarity, reverse=True)
        topk_results = results[:topk]

        return topk_results

class ReSpeaker:
    def __init__(self, topic_name: str = "/voice/text", speaker_topic: str="/speaker/say", nlp_model_name: str = "en_core_web_sm"):
        self.events = []
        self.isready = False
        self.nlp = NLP(model_name=nlp_model_name)

        logger.info("Initializing ReSpeaker")
        self.subscriber = rospy.Subscriber(topic_name, Voice, self.callback)
        self.publisher = rospy.Publisher(speaker_topic, String, queue_size=10)
        rospy.sleep(1)
        self.isready = True
        logger.success("ReSpeaker is ready!")

    def callback(self, msg: Voice):
        nlp_result = self.nlp.check_sentence(msg.text)
        direction = msg.direction
        time = msg.time
        self.events.append(ReSpeaker_event(nlp=nlp_result, time=time, direction=direction, text=msg.text))
        logger.debug(f"ReSpeaker received sentence \"{msg.text}\", direction={msg.direction}, time={msg.time}")

    def get_events(self, batch_size=-1) -> List[ReSpeaker_event]:
        if batch_size == -1:
            tmp = self.events.copy()
            self.events.clear()
            return tmp
        tmp = self.events[-batch_size:]
        self.events = self.events[:-batch_size]
        return tmp

    def say(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node("RosCode", anonymous=True)

    respeaker = ReSpeaker("/voice/text")
    sent_ids = [
        respeaker.nlp.add_sentence("take my bag"),
        respeaker.nlp.add_sentence("go to kitchen"),
        respeaker.nlp.add_sentence("go to bedroom"),
        respeaker.nlp.add_sentence("switch on the light")
    ]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

        for event in respeaker.get_events():
            print(event.nlp)
            print(event.nlp[0].matched_sentence)


