# ROS-group-3-OneShot 🎯

Hey there, it's me, One-Shot! Your ultimate interview practice sidekick! 🌟 With my awesome superpowers, I can boost your confidence by analyzing your facial expressions, fact-check your statements, and give you top-notch feedback for improvement! 💪😃
</br>
</br>

## Project Setup 🚀

This will install the python environment, necessary dependencies and catkin workspace for the project. Open a terminal and enter the commands below line by line.

Setup the Python environment and making it default:
- <code>pyenv install 3.9.2</code> </br>
- <code>pyenv shell 3.9.2</code> </br>
- <code>echo "pyenv shell 3.9.2" >> ~/.bashrc</code> </br>


Installing the necessary Python dependencies: </br>
- <code>pip3 install opencv-python deepface SpeechRecognition openai tf-keras numpy</code> </br>
- <code>pip3 install langchain langchain-core langchain-openai langchain-community tavily-python</code> </br>
- <code>pip3 install torch bert-extractive-summarizer ffmpeg pyaudio rospkg</code> </br>
- <code>pip3 cache purge</code> </br>


Create your catkin workspace for the project: </br>
- <code>mkdir catkin_ws_2</code> </br>
- <code>cd catkin_ws_2</code> </br>
- <code>mkdir src</code> </br>
- <code>catkin_make</code> </br>
- <code>echo "source ~/catkin_ws_2/devel/setup.bash" >> ~/.bashrc </code> </br>
- <code>cd src</code> </br>
- <code>catkin_create_pkg group3 roscpp rospy std_msgs</code> </br>
- <code>cd ..</code> </br>
- <code>catkin_make</code> </br>
- <code>cd src/group3/src</code> </br>


Place the 5 python files and making them executable: </br>
- <code>TODO: place the 5 python files at this directory</code> </br>
- <code>chmod +x *</code> </br>

</br>
This project will be using both OpenAI and Tavily's APIs for <code>gpt-3.5-turbo</code> model inferencing and <code>Tavily Web Browser tool</code> for the model. If you already have the API keys for both OpenAI and Tavily, you may head to <code>text_eval_node.py</code> and place your API keys there. If not, please head to <a href='https://platform.openai.com/docs/api-reference/introduction'>platform.openai.com</a> and <a href='https://app.tavily.com/home'>app.tavily.com</a> to obtain your API keys.



</br>
</br>

## Run the Project 🏎️💨

To run the project, open 6 terminals and enter the below in all of them: </br>
- <code>cd /home/mustar/catkin_ws_2/src/group3/src</code> </br>

Then in each of the terminals, run the different nodes:
- <code>roscore</code> </br>
- <code>python main.py</code> </br>
- <code>python main_node.py</code> </br>
- <code>python record_video_node.py</code> </br>
- <code>python deepFace_node.py</code> </br>
- <code>python text_eval_node.py</code> </br>

When you're done, head to the terminal where you ran <code>python main.py</code>, enter the interview question that you would like to practice there, and the duration of your response (in Seconds!). You should hear it talk and you're good to go!

Demo video of the project running can be found <a href='https://drive.google.com/file/d/1PjMC9hnNe6O7jbpS3IO_gdXaqJSAgzEx/view?usp=sharing'>here</a>!


</br>
</br>

## About the Project 🐛🔍

🤖 This project utilizes 3 ML models, in which 2 will be running locally (but don't worry they won't be large), and 1 from the cloud:
- **GPT-2 Summarizer model** for text content summarising (local)
- **DeepFace Emotion Recognition model** for facial confidence score calculation (local)
- **OpenAI GPT-3.5-Turbo model** for factual validation via Tavily web browsing, and evaluation for feedback (OpenAI API)

</br>

🔧 Core technologies included in this project are:
- <a href='https://tavily.com'>Tavily</a> web browsing tool for LLMs
- <a href='https://www.langchain.com/langchain'>Langchain</a> for integration of GPT-3.5-Turbo with Tavily
- <a href='https://pypi.org/project/gTTS/'>Google's Speech-to-Text & Text-to-Speech service</a> (gTTS)
- LLMs (<a href='https://pypi.org/project/bert-extractive-summarizer/'>local GPT-2 Summarizer model</a>, <a href='https://platform.openai.com/docs/api-reference/introduction'>OpenAI GPT-3.5-Turbo model</a>)
- Image Processing Models (<a href='https://pypi.org/project/deepface/'>DeepFace Emotion Recognition model</a>)


</br>
</br>
</br>
Happy Coding! 🎉 </br>
~ Group 3 ~
