$.ajax({
    url: "http://vm-iot-school-2020-1.westeurope.cloudapp.azure.com:80/api/telemetry/measurement",
    data: {
        "DeviceId": '1',
        "SensorName": "Temperature", 
        "dateFrom": "2021-02-01",
        "dateTo": "2022-02-28"
    },
    cache: false,
    type: "GET",
    success: function (response) {
        values = [];
        captions = [];
        for (var i = 0; i < response.length; i++) {
            captions[i] = response[i]["CreatedOn"];
            values[i] = response[i]["SensorValue"];
        }
 
        var ctx = document.getElementById("Mychart");
        var mychart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: captions,
                datasets: [
                    {
                        data: values,
                        label: "Temperature",
                        borderColor: "#3e95cd",
                        fill: false
                    },
                ]
            }
        });
    },
    error: function (xhr) {
        console.log("Ajax error!");
        console.log(xhr);
    }
});