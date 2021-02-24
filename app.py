from flask import Flask
from flask import jsonify
from flask_mysqldb import MySQL

mysql = MySQL()
app = Flask(__name__)

#MySQL configuration
app.config['MYSQL_USER'] = 'booksuser'
app.config['MYSQL_PASSWORD'] = 'ThePass11$Dot'
app.config['MYSQL_DB'] = 'booksDB'
app.config['MYSQL_HOST'] = 'localhost'
mysql.init_app(app)

books = [{'name': 'Snow White', 'author': 'Grimm brothers'},
         {'name': 'If it bleeds', 'author': 'Stephen King'}
]


@app.route('/', methods=['GET'])
def hello_world():
    return 'Hello world!'

@app.route("/api/books", methods=['GET'])
def return_all():
    conn = mysql.connect()
    cursor = conn.cursor()

    cursor.execute("SELECT Name, Author FROM book")
    rows = cursor.fetchall()

    return jsonify({'rows': rows})

@app.route("/api/books/titles", methods=['GET'])
def return_titles():
    titles = []
    for book in books:
        titles.append(book['name'])
    return jsonify({'titles': titles})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port='80')
