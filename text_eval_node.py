#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import json
import speech_recognition as sr
from summarizer import TransformerSummarizer

import datetime
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import MessagesPlaceholder
from langchain.agents import create_openai_functions_agent
from langchain_community.tools.tavily_search import TavilySearchResults
from langchain.agents import AgentExecutor


class TextEval:
    def __init__(self):
        rospy.init_node('text_eval_node')
        
        # Local GPT2 config
        self.SUMM_TF_TYPE      = "GPT2" # genre
        self.SUMM_TF_MODEL_KEY = "gpt2" # specifics: gpt2 | gpt2-medium | gpt2-large | distilgpt2
        self.SUMM_TF_MIN_LEN   = 0.3    # summarizer transformer response minimum length, 30% of original audio text length
        
        # openai & langchain config
        self.OPENAI_API_KEY    = '<place-OPENAI-API-KEY-here>'
        self.TAVILY_API_KEY    = '<place-TAVILY-API-KEY-here>' # browser for openai model
        self.OPENAI_MODEL      = 'gpt-3.5-turbo'
        self.VERBOSE           = True
        self.PERSONA_INTERVIEWER = "Please adopt the persona of a Human Resource Manager and an interviewer. Your task is to review an interviwee's performance."
        self.MODEL_TASK = "You will be provided with the interview question, the interviewee's facial confidence score for the interview session, as well as his/her speech texts. The confidence score will range from 0 to 100. Based on the question, confidence score and speech, please provide a short and concise feedback and possible room for improvement to the interviewee. Provide examples on how it can be improved. You may reference the interviewee's speech texts. The feedback must be highlight the important points, but be short and concise. Please sturcture the feedback in a way that your are talking to the interviewee."
        self.MODEL_SUPERPOWERS = "You will be provided with the ability to browse the internet. You may use this to check or identify the accuracy of the speech texts of the interviewee."
        
        # init models & agents
        self.transformer_model = TransformerSummarizer(transformer_type=self.SUMM_TF_TYPE ,transformer_model_key=self.SUMM_TF_MODEL_KEY)
        self.agent_executor    = self.init_functional_lc_model(system_prompt=f"{self.PERSONA_INTERVIEWER} {self.MODEL_TASK} {self.MODEL_SUPERPOWERS}", 
                                                    model=self.OPENAI_MODEL, openai_api_key=self.OPENAI_API_KEY, tavily_api_key=self.TAVILY_API_KEY, verbose=self.VERBOSE)
        
        # subscribers & publishers
        self.sub_p5_text_eval = rospy.Subscriber("/p5_text_eval", String, self.handle_payload)
        self.pub_p6_results   = rospy.Publisher("/p6_results",   String, queue_size=10)


    def handle_payload(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            # Step 3: audio to text
            self.QUESTION = str(data['question'])
            self.CONF_SCORE = int(data['conf'])
            
            text = self.audio_to_text(data['audio'])
            
            # Step 4: summarize text
            min_text_len = int(self.SUMM_TF_MIN_LEN * len(text))
            min_text_len = min_text_len if min_text_len > 0 else 1
            summary = self.summarize_text(text, min_length=min_text_len, transformer_type=self.SUMM_TF_TYPE, transformer_model_key=self.SUMM_TF_MODEL_KEY)
            
            # Step 5: eval and provide feedback
            feedback = self.eval_and_feedback(agent_executor=self.agent_executor, question=self.QUESTION, response=text, conf_score=self.CONF_SCORE)

            payload = {'conf': int(self.CONF_SCORE),
                       'summary': str(summary),
                       'feedback': str(feedback)}
            
            # =======================================
            
            payload = json.dumps(payload)
            self.pub_p6_results.publish(payload)
        except:
            rospy.loginfo("[text_eval_node.py] Error parsing json payload.")


    # run google's text-2-audio engine (local)
    def audio_to_text(self, audio_file):
        r = sr.Recognizer()
        with sr.AudioFile(audio_file) as source:
            audio = r.record(source)
            text = r.recognize_google(audio)
        return text


    def summarize_text(self, input="", min_length=60, transformer_type="GPT2", transformer_model_key='gpt2'):
        re = None
        if len(input) > 0:
            re = ''.join(self.transformer_model(input, min_length=min_length))
        return re


    def init_functional_lc_model(self, system_prompt='', model='gpt-3.5-turbo', openai_api_key='', tavily_api_key='', verbose=False):
        llm = ChatOpenAI(openai_api_key=openai_api_key, model=model)
        prompt_template = ChatPromptTemplate.from_messages(
            [
                ("system", system_prompt),
                MessagesPlaceholder("chat_history", optional=True),
                ("human", "{input}"),
                MessagesPlaceholder("agent_scratchpad"),
            ]
        )

        os.environ["TAVILY_API_KEY"] = tavily_api_key
        browser_tool = TavilySearchResults()
        
        tools = [browser_tool]
        agent = create_openai_functions_agent(llm, tools, prompt_template)
        agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=verbose)
        return agent_executor


    def eval_and_feedback(self, agent_executor:AgentExecutor, question, response, conf_score):
        formatted_input = f"The current datetime is {datetime.datetime.now().strftime('%A, %B %d, %Y %H:%M:%S')}. The interview question is: {question}. The interviewee's response is as follows: {response}. The interviewee's facial confidence score is: {conf_score} out of 100."
        chat_history = []  # not required here
        try:
            response = agent_executor.invoke({
                "chat_history": chat_history,
                "input": formatted_input
            })
            output = response['output']
            return output
        except:
            return None



if __name__ == "__main__":
    try:
        TextEval()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[text_eval_node.py] Process Terminated.")
        