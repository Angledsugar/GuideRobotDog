# # importing the module
# import speech_recognition as sr
# # define the recognizer
# r = sr.Recognizer()
# # define the audio file
# audio_file = sr.AudioFile('test.wav')
# # speech recognition
# with audio_file as source: 
#    r.adjust_for_ambient_noise(source) 
#    audio = r.record(source) 
# result = r.recognize_google(audio)
# # exporting the result 
# with open('test.txt',mode ='w') as file: 
#    file.write("Recognized text:") 
#    file.write("\n") 
#    file.write(result) 
#    print("ready!")

import speech_recognition as sr
import pyaudio

r = sr.Recognizer()

with sr.Microphone() as source:
    print('Speack Anything :')
    audio = r.listen(source)
    
    try:
        text = r.recognize_google(audio)
        print('You said : {}'.format(text))
    except:
        print('Sorry could not recignize your voice')

#install method
#pip3 install SpeechRecognition
#pip3 install PyAudio
