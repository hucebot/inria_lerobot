from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageColor
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
import sys

class StreamDeck:
    def __init__(self, robot=None):
        self.stream_deck = None
        self.device_manager = DeviceManager()

        self.key_width = 100
        self.key_height = 100
        self.color_text = "#000000"
        self.background_color_active = "#27a007"
        self.background_color_inactive = "#bababa"
        self.background_color_red = "#ff0000"
        self.background_color_orange = "#ff7f00"

        self.is_recording = False
        self.robot = robot

        try:
            self.stream_deck = self.device_manager.enumerate()[0]
            self.stream_deck.open()
            self.stream_deck.reset()
            self.stream_deck.set_brightness(100)
        
        except Exception as e:
            print(f"Stream Deck not detected: {e}")
            self.cleanup()
            sys.exit(1)

        self.column, self.row = self.stream_deck.key_layout()
        self.background_image = Image.new("RGB", (self.key_width, self.key_height), color=ImageColor.getrgb("#000000"))
        self.initialize_buttons()

        self.stream_deck.set_key_callback(self.on_key_change)

    def on_key_change(self, deck, key, state):
        ''' Function when a key is pressed or released'''
        if state:
            if key == self.start_recording_button and not self.is_recording:
                self.is_recording = True
                print("Start recording")
                self.robot.toggle_recording()
                self.create_button(self.start_recording_button, "STOP RECORDING", self.background_color_active)

            elif key == self.start_recording_button and self.is_recording:
                self.is_recording = False
                print("Stop recording")
                self.robot.toggle_recording()
                self.create_button(self.start_recording_button, "START RECORDING", self.background_color_inactive)

            elif key == self.delete_bag_button:
                print("Delete last bag")
                self.robot.delete_last_record()
                self.create_button(self.delete_bag_button, "DELETE LAST", self.background_color_red)

        else:
            self.create_button(self.delete_bag_button, "DELETE LAST", self.background_color_inactive)

    def cleanup(self):
        ''' Cleanup function to close the stream deck '''
        if self.stream_deck:
            try:
                self.stream_deck.reset()
            except Exception:
                pass
            try:
                self.stream_deck.close()
            except Exception:
                pass
            self.stream_deck = None

    def create_button(self, position, label, background_color):
        ''' Function to create a button on the stream deck '''
        image = self.background_image.copy()
        label = label.upper()
        x_pos = 0
        y_pos = 40

        if " " not in label:
            x_pos = 50 - (len(label) * 5)
        else:
            list_words = label.split(" ")
            label = ""
            for word in list_words:
                len_word = len(word)
                if len_word > 7:
                    label += " "*11 + word + "\n"
                elif len_word > 5:
                    label += " "*15 + word + "\n"
                else:
                    label += " "*18 + word + "\n"

        draw = ImageDraw.Draw(image)
        draw.rectangle([(0, 0), (self.key_width, self.key_height)], fill=ImageColor.getrgb(background_color))

        draw.text((x_pos, y_pos), label, fill=ImageColor.getrgb(self.color_text))
        
        key_image = PILHelper.to_native_format(self.stream_deck, image)
        self.stream_deck.set_key_image(position, key_image)
        return position

    def initialize_buttons(self):
        ''' Function to initialize the buttons on the stream deck '''
        # START RECORDING BUTTON
        self.start_recording_button = (0, 1)
        self.start_recording_button = self.start_recording_button[0] * self.row + self.start_recording_button[1]
        self.create_button(self.start_recording_button, "START RECORDING", self.background_color_inactive)

        # DELETE LAST BUTTON
        self.delete_bag_button = (0, 3)
        self.delete_bag_button = self.delete_bag_button[0] * self.row + self.delete_bag_button[1]
        self.create_button(self.delete_bag_button, "DELETE LAST", self.background_color_inactive)