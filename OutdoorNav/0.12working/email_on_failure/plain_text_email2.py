#!/usr/bin/env python

import smtplib
from email.message import EmailMessage

password = "quya pzra arwe uogo"
variable = 2
string = "nathan"

def sendEmail():
    msg = EmailMessage()
    msg.set_content('This is my message ' + string)

    msg['Subject'] = 'Subject ' + str(variable) + " " + string
   
    msg['From'] = "huskyobserver@gmail.com" 
    msg['To'] = "huskyobserver+python@gmail.com"

    server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
    server.login("huskyobserver@gmail.com", password)
    server.send_message(msg)
    server.quit()


if __name__ == '__main__':
     sendEmail()

     exit()