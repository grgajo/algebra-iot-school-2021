from flask import Flask
app = Flask(__name__)

books = [{'name': 'Snow White', 'author': 'Grimm brothers'},
         {'name': 'If it bleeds', 'author': 'Stephen King'}
]



@app.route('/', methods=['GET'])
def hello_world():
    return 'Hello world!'

if __name__ == "__main__":
    app.run(host='0.0.0.0', port='80')
