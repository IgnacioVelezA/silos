import os
import sys
import telebot

if __name__ == '__main__':

    IP = sys.argv[1]

    BOT_TOKEN = os.environ.get('BOT_TOKEN')

    bot = telebot.TeleBot(BOT_TOKEN)
    
    @bot.message_handler(commands=['ip'])
    def send_ip(message):
        bot.reply_to(message, f'Current IP: {IP}')

    bot.infinity_polling()
