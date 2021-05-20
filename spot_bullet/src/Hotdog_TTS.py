#pip install gtts
#pip install playsound

from gtts import gTTS
import playsound

# text = '안녕하세요. 저는 안내견 Hotdog 입니다. Hotdog로 불러주세요.'
# tts_kr = gTTS(text= text,lang='ko')
# tts_kr.save("Hotdog_voice_1.mp3")
# playsound.playsound('Hotdog_voice_1.mp3')

# text = '하네스를 오른손으로 잡고 편안한 상태로 서 있어주세요.'
# tts_kr = gTTS(text= text,lang='ko')
# tts_kr.save("Hotdog_voice_2.mp3")
# playsound.playsound('Hotdog_voice_2.mp3')

# text = '환경설정이 완료되었습니다. 이제 그럼 출발할까요?'
# tts_kr = gTTS(text= text,lang='ko')
# tts_kr.save("Hotdog_voice_3.mp3")
# playsound.playsound('Hotdog_voice_3.mp3')

# text = '긴급상황 발생! 잠시 멈춰주세요!'
# tts_kr = gTTS(text= text,lang='ko')
# tts_kr.save("Hotdog_voice_4.mp3")
# playsound.playsound('Hotdog_voice_4.mp3')

text = 'Hello? I am Hotdog. Call me Hotdog'
tts_kr = gTTS(text= text,lang='en')
tts_kr.save("Hotdog_voice_1.mp3")
playsound.playsound('Hotdog_voice_1.mp3')

text = 'Please stand with the harness in your right hand.'
tts_kr = gTTS(text= text,lang='en')
tts_kr.save("Hotdog_voice_2.mp3")
playsound.playsound('Hotdog_voice_2.mp3')

text = 'Environment setup is completed. Lets start!'
tts_kr = gTTS(text= text,lang='en') 
tts_kr.save("Hotdog_voice_3.mp3")
playsound.playsound('Hotdog_voice_3.mp3')

text = 'Stop! Emergency situation occurs! Please stop!'
tts_kr = gTTS(text= text,lang='en')
tts_kr.save("Hotdog_voice_4.mp3")
playsound.playsound('Hotdog_voice_4.mp3')

# f = open(tempFileName,'wb')             
# tts_kr.write_to_fp(f)    # 한글로 한번 말하기
# f.close()