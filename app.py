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
    return jsonify({'books': books})

@app.route("/api/books/titles", methods=['GET'])
def book_titles():
    titles = []
    for book in books:
        titles.append(book['name'])
    return jsonify({'titles': titles})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port='80')
