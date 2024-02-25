# Import the following modules
from pushbullet import PushBullet
from pywebio.input import *
from pywebio.output import *
from pywebio.session import *
import time

class Notify:
    def Not(data = "Fault Found", text, access_key):
# Get the access token from Pushbullet.com
#         data = data1
#         access_token = access_key
        pb = PushBullet(access_key)

        text = text
        # Taking large text input from the user
        # text = textarea(
        # "Text", rows=3, placeholder="Write something...",
        # required=True)

        # Get the instance using access token


        # Send the data by passing the main title
        # and text to be send
        push = pb.push_note(data, text)

        hold()

if __name__ ==" __main__":
    Notify.Not( text = "Hooo", "o.4ufPmlLMHkASCG5OHsaUSSPy5mzojEv4",)
