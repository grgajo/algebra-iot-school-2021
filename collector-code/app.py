from flask import Flask, render_template, json, request, redirect, session, jsonify
from flask import Markup
from flask_mysqldb import MySQL
from flask import session, request
from flask import make_response, abort
from datetime import datetime

mysql = MySQL()
app = Flask(__name__)

# MySQL configurations
app.config['MYSQL_USER'] = 'telemetry-user'
app.config['MYSQL_PASSWORD'] = 'ZAP145!2'
app.config['MYSQL_DB'] = 'telemetryDB'
app.config['MYSQL_HOST'] = 'localhost'
mysql.init_app(app)


@app.route('/')
def main():
    return render_template('index.html')

@app.route('/temperature')
def chartTemperature():
    return render_template('temperature.html')

@app.route('/heartrate')
def chartHeartrate():
    return render_template('heartrate.html')

@app.route('/battery')
def chartBattery():
    return render_template('battery.html')


@app.route('/map')
def chartMap():
    return render_template('map.html')

@app.route('/api/telemetry/measurement', methods=['POST'])
def post_measurement():
    if not request.json or not 'DeviceId' in request.json:
        abort(400)
    else:
        content = request.json
    measurement = {
        'DeviceId' : content['DeviceId'],
        'SensorName' : content['SensorName'],
        'SensorValue' : content['SensorValue'],
        'CreatedOn' : datetime.now()
    }
    return add_measurement(measurement)

@app.route('/api/telemetry/measurement', methods=['GET'])
def get_measurement():
    content = request.args
    if not content or not content.get('DeviceId') or not content.get('SensorName'):
        abort(400)
    else:
        conn = mysql.connect
        cursor =conn.cursor()

        cmd = "SELECT DeviceId, CreatedOn, SensorValue FROM Measurement WHERE SensorName = %s AND CreatedOn >= %s AND CreatedOn < %s"
        params = (content.get('SensorName'), content.get('dateFrom'), content.get('dateTo'))
        cursor.execute(cmd, params)
        rows = cursor.fetchall()

        data = list()

        for row in rows:
            data.append({"DeviceId": row[0], "CreatedOn": row[1], "SensorValue": str(row[2])})

        return jsonify(data)

@app.route('/api/telemetry/devices', methods=['GET'])
def get_devices():
    conn = mysql.connect
    cursor =conn.cursor()

    cursor.execute("SELECT DeviceId, Name, latitude, longitude FROM Device")
    rows = cursor.fetchall()

    data = list()

    for row in rows:
        data.append({"DeviceId": row[0], "Name": row[1], "latitude": str(row[2]), "longitude": str(row[3])})

    return jsonify(data)

def add_measurement(data):
    conn = mysql.connect
    cursor =conn.cursor()
    try:
        cmd = "INSERT INTO Measurement (DeviceId, SensorName, SensorValue, CreatedOn) VALUES (%s, %s, %s, %s)"
        params = (data["DeviceId"], data["SensorName"], data["SensorValue"], data["CreatedOn"])
        cursor.execute(cmd, params)
        conn.commit()
        cursor.close()
        conn.close()
    except Exception as e:
        print ("Error: unable to fetch items", e)
    return "200"


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80)

