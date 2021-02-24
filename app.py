from flask import Flask
from flask import jsonify
app = Flask(__name__)

books = [{'name': 'Snow White', 'author': 'Grimm brothers'},
         {'name': 'If it bleeds', 'author': 'Stephen King'}
]



@app.route('/', methods=['GET'])
def hello_world():
    return 'Hello world!'

@app.route("/api/books", methods=['GET'])
def return_all():
    return jsonify({'books': books.get('name')})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port='80')
