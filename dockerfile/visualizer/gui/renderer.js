// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// No Node.js APIs are available in this process because
// `nodeIntegration` is turned off. Use `preload.js` to
// selectively enable features needed in the rendering
// process.

// const { join } = require('path')
// require(join(appRoot, 'rclnodejs'))


window.addEventListener('DOMContentLoaded', () => {
    const ws = require('electron').remote.getGlobal('sharedObject').ws

    temp_array = ['temperature'].concat(new Array(100).fill(0))
    hum_array = ['humidity'].concat(new Array(100).fill(0))

    var chart = c3.generate({
        bindto: '#chart',
        point: {
            show: false
        },
        transition: {
            duration: 100
        },
        data: {
          columns: [
            temp_array,
            hum_array
          ],
          axes: {
            humidity: 'y2'
          }
        },
        axis: {
            y: {
                max: 50,
                min: 0,
                label: { 
                  text: 'Temperature (ºC)',
                  position: 'outer-middle'
                }
            },
            y2: {
                max: 100,
                min: 0,
                show: true,
                label: {
                    text: 'Humidity (%)',
                    position: 'outer-middle'
                  }
            }
        }
    });

    const label = document.getElementById('connection')
    var last_message = null
    setInterval(_ => {
        const current_time = new Date()
        if (last_message && (current_time - last_message) < 3000) {
            label.style.color = "#5cb85c"
            label.innerHTML = "Connected"
        }else{
            label.style.color = "#d9534f"
            label.innerHTML = "Not connected"
        }
    })


    ws.on('message', (data) => {
        data = JSON.parse(data)
        if (data.humidity) {
            hum_array.push(data.humidity)
            hum_array.splice(1,1)
        }

        if (data.temperature) {
            temp_array.push(data.temperature)
            temp_array.splice(1,1)
        }

        chart.load({
            columns: [
                temp_array,
                hum_array
            ]
        });

        last_message = new Date()
    });
})
  
