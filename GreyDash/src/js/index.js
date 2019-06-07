/**
 * Pull the JSON config file.
 */
var config
$.getJSON('js/configlisteners.json', (json) => {
    config = json
    render()
})

/**
 * Define UI Elements
 */
const ui = {
    misc: {
        timer: document.getElementById('timer'),
        robotState: document.getElementById('robotState').firstChild,
        autoSelect: document.getElementById('autoSelect'),
        camera: {
            cameraImg: document.getElementById('camera'),
            container: document.getElementById('cameraContainer')
        }
    },
    debugCharts: document.getElementById('debugCharts'),
    indicators: document.getElementById('indicators'),
    debugIndicators: document.getElementById('debugIndicators'),
    custom: {
        drive: {
            gyro: {
                container: document.getElementById('gyro'),
                val: 0,
                visualVal: 0,
                gyroDial: document.getElementById('gyroDial'),
                gyroNumber: document.getElementById('gyroNumber')
            },
            debugGyro: {
                container: document.getElementById('debugGyro'),
                val: 0,
                visualVal: 0,
                debugGyroDial: document.getElementById('debugGyroDial'),
                debugGyroNumber: document.getElementById('debugGyroNumber')
            }
        }
    }
}

/**
 * Help dialog.
 */

const helpDialog = document.getElementById('helpDialog')
const helpCloseButton = document.getElementById('helpCloseButton')

// Function for hiding the connect box
document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') {
        helpDialog.close()
    }
})

if (!helpDialog.showModal) {
    dialogPolyfill.registerDialog(loginDialog)
}

helpCloseButton.addEventListener('click', () => {
    helpDialog.close()
})

function openPage (pageName) {
    var tabs = document.getElementsByClassName('tabcontent')
    for (var i = 0; i < tabs.length; i++) {
        tabs[i].style.display = 'none'
    }
    document.getElementById(pageName).style.display = 'block'
}

let graphIntervals = [];

function render () {
    console.log('rendering')
    // Set Chart Defaults
    for (let i = 0; i < config.charts.length; i += 1) {
        if (config.charts[i].settings.tooltip === undefined) {
            console.warn(`Setting tooltip for '${config.charts[i].title}' to false`)
            config.charts[i].settings.tooltip = false
        }

        if (config.charts[i].settings.minValue === undefined) {
            console.warn(`Setting min for '${config.charts[i].title}' to 0`)
            config.charts[i].settings.minValue = 0
        }

        if (config.charts[i].settings.maxValue === undefined) {
            console.warn(`Setting max for '${config.charts[i].title}' to 50`)
            config.charts[i].settings.maxValue = 50
        }

        if (config.charts[i].settings.interpolation === undefined) {
            console.warn(`Setting interpolation for '${config.charts[i].title}' to step`)
            config.charts[i].settings.interpolation = 'step'
        }

        if (config.charts[i].settings.show === undefined) {
            console.warn(`Setting show for '${config.charts[i].title}' to true`)
            config.charts[i].settings.show = true
        }

        if (config.charts[i].settings.millisPerPixel === undefined) {
            console.warn(`Setting millisPerPixel for ${config.charts[i].title} to 20`)
            config.charts[i].settings.millisPerPixel = 20
        }
    }

    // Set Indicator Defaults
    for (let i = 0; i < config.indicators.length; i += 1) {
        if (config.indicators[i].settings.fixedDecimals === undefined) {
            config.indicators[i].settings.fixedDecimals = true
        }

        if (config.indicators[i].settings.debug === undefined && config.indicators[i].settings.default === undefined) {
            console.log(`Indicator ${i} has neither debug or default specified. Using debug.`)
            config.indicators[i].settings.debug = true
            config.indicators[i].settings.default = false
        } else if (config.indicators[i].settings.debug === false && config.indicators[i].settings.default === false) {
            console.log(`Indicator ${i} has debug and default set to false. Set show to false to hide instead.`)
            config.indicators[i].settings.debug = true
            config.indicators[i].settings.default = false
        } else if (config.indicators[i].settings.debug && config.indicators[i].settings.default) {
            config.indicators[i].settings.both = true
        } else if (config.indicators[i].settings.debug) {
            config.indicators[i].settings.default = false
        } else if (config.indicators[i].settings.default) {
            config.indicators[i].settings.debug = false
        }

        if (config.indicators[i].settings.show === undefined) {
            config.indicators[i].settings.show = true
        }
    }

    // Chart Listeners
    for (let i = 0; i < config.charts.length; i += 1) {
        if (config.charts[i].settings.show) {
            config.charts[i].div = document.createElement('div')
            config.charts[i].div.setAttribute('class', 'chart')

            config.charts[i].displayTitle = document.createElement('span')
            config.charts[i].displayTitle.innerText = `${config.charts[i].title}`

            config.charts[i].displayChart = document.createElement('canvas')
            config.charts[i].displayChart.setAttribute('class', 'responsiveChart')
            config.charts[i].displayChart.setAttribute('name', `chart${i}`)

            config.charts[i].chart = new SmoothieChart({
                responsive: 'true',
                interpolation: config.charts[i].settings.interpolation,
                tooltip: config.charts[i].settings.tooltip,
                minValue: config.charts[i].settings.minValue,
                maxValue: config.charts[i].settings.maxValue,
                millisPerPixel: config.charts[i].settings.millisPerPixel
            })

            config.charts[i].lines = {}

            for (let line = 0; line < config.charts[i].keys.length; line += 1) {
                const colors = ['red', 'blue', 'yellow', 'violet', 'orange', 'indigo']
                config.charts[i].lines[line] = new TimeSeries()
                config.charts[i].chart.addTimeSeries(config.charts[i].lines[line], {
                    strokeStyle: colors[line],
                    lineWidth: 4
                })
                NetworkTables.addKeyListener(config.charts[i].keys[line], (function bindIndicator (idx) {
                    return (key, value) => {
                        const intervalKey = `${idx}${line}`
                        if (graphIntervals[intervalKey] != null) {
                            clearInterval(graphIntervals[intervalKey])
                        }

                        graphIntervals[intervalKey] = setInterval(() => {
                            config.charts[idx].lines[line].append(new Date().getTime(), value)
                        }, 20)
                    }
                }(i)), true)
            }

            config.charts[i].chart.streamTo(config.charts[i].displayChart, 0)

            config.charts[i].div.appendChild(config.charts[i].displayTitle)
            config.charts[i].div.appendChild(document.createElement('br'))
            config.charts[i].div.appendChild(config.charts[i].displayChart)

            ui.debugCharts.appendChild(config.charts[i].div)
        }
    }

    // Indicator Listeners
    for (let i = 0; i < config.indicators.length; i += 1) {
        if (config.indicators[i].settings.show) {
            let unitvalue
            config.indicators[i].div = document.createElement('div')
            config.indicators[i].div.setAttribute('id', `indicator${i}`)

            config.indicators[i].displayTitle = document.createElement('span')
            config.indicators[i].displayTitle.innerText = `${config.indicators[i].title}: `

            config.indicators[i].displayValue = document.createElement('span')

            NetworkTables.addKeyListener(config.indicators[i].key, (function bindIndicator (idx) {
                return (key, value) => {
                    if (config.indicators[idx].settings.fixedDecimals === true) {
                        unitvalue = `${value.toFixed(2)} ${config.indicators[idx].unit}`
                    } else {
                        unitvalue = `${value} ${config.indicators[idx].unit}`
                    }
                    config.indicators[idx].displayValue.innerText = unitvalue
                }
            }(i)))

            config.indicators[i].displayTitle.appendChild(config.indicators[i].displayValue)
            config.indicators[i].div.appendChild(config.indicators[i].displayTitle)
            ui.indicators.appendChild(config.indicators[i].div)

            if (config.indicators[i].settings.default) {
                ui.indicators.appendChild(config.indicators[i].div)
            } else if (config.indicators[i].settings.bothTabs) {
                ui.indicators.appendChild(config.indicators[i].div)
                ui.debugIndicators.appendChild(config.indicators[i].div.cloneNode(true))
            } else {
                ui.debugIndicators.appendChild(config.indicators[i].div)
            }
        }
    }

    /**
     * Custom Listeners
     */

    // Gyro rotation
    const updateGyro = (key, value) => {
        ui.custom.drive.gyro.val = value
        ui.custom.drive.gyro.visualVal = Math.floor(ui.custom.drive.gyro.val)
        ui.custom.drive.gyro.visualVal %= 360

        if (ui.custom.drive.gyro.visualVal < 0) {
            ui.custom.drive.gyro.visualVal += 360
        }

        // ui.custom.drive.gyro.gyroDial.style.transform = `rotate(-${ui.custom.drive.gyro.visualVal}deg)`
        ui.custom.drive.gyro.gyroDial.setAttribute('transform', `rotate(-${ui.custom.drive.gyro.visualVal} 0 -83)`)
        ui.custom.drive.gyro.gyroNumber.innerHTML = `${ui.custom.drive.gyro.visualVal}º`
    }

    const updateDebugGyro = (key, value) => {
        ui.custom.drive.debugGyro.val = value
        ui.custom.drive.debugGyro.visualVal = Math.floor(ui.custom.drive.debugGyro.val)
        ui.custom.drive.debugGyro.visualVal %= 360

        if (ui.custom.drive.debugGyro.visualVal < 0) {
            ui.custom.drive.debugGyro.visualVal += 360
        }

        ui.custom.drive.debugGyro.debugGyroDial.setAttribute('transform', `rotate(-${ui.custom.drive.debugGyro.visualVal} 0 -83)`)
        ui.custom.drive.debugGyro.debugGyroNumber.innerHTML = `${ui.custom.drive.debugGyro.visualVal}º`
    }

    NetworkTables.addKeyListener('/SmartDashboard/drive/gyro/angle', updateGyro)
    NetworkTables.addKeyListener('/SmartDashboard/drive/gyro/angle', updateDebugGyro)

    NetworkTables.addKeyListener('/SmartDashboard/misc/timer', (key, value) => {
        console.log(value)

        // Make sure timer is reset to black when it starts
        ui.misc.timer.style.color = '#000000'

        // Minutes (m) is equal to the total seconds divided by sixty with the decimal removed.
        const m = Math.floor(value / 60)

        // Create seconds number that will actually be displayed after minutes are subtracted
        let visualS = (value % 60)

        // Add leading zero if seconds is one digit long, for proper time formatting.
        visualS = visualS < 10 ? `0${visualS}` : visualS

        if (value <= 15) {
            // Flash timer if less than 15 seconds left
            ui.misc.timer.style.color = (value % 2 === 0) ? '#FF3030' : 'transparent'
        } else if (value <= 30) {
            // Solid red timer when less than 30 seconds left.
            ui.misc.timer.style.color = '#FF3030'
        }

        ui.misc.timer.innerHTML = `${m}:${visualS}`
        NetworkTables.putValue(key, false)
    })

    function dbTableHandler (key, value) {
        const cell = document.getElementById(key)
        cell.innerText = value
    }

    NetworkTables.addKeyListener('/SmartDashboard/DB/String 0', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 1', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 2', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 3', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 4', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 5', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 6', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 7', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 8', dbTableHandler)
    NetworkTables.addKeyListener('/SmartDashboard/DB/String 9', dbTableHandler)

    /**
     * Connection dialog.
     */

    const connectionDialog = document.getElementById('connectionDialog')
    const connectionCloseButton = document.getElementById('connectionCloseButton')

    // Function for hiding the connect box
    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
            connectionDialog.close()
        }
    })

    if (!connectionDialog.showModal) {
        dialogPolyfill.registerDialog(loginDialog)
    }

    connectionCloseButton.addEventListener('click', () => {
        connectionDialog.close()
    })

    /**
     * Function to be called when robot connects
     * @param {boolean} connected
     */
    // Set function to be called when robot dis/connects
    NetworkTables.addRobotConnectionListener((connected) => {
        const state = connected ? 'Robot connected!' : 'Robot disconnected.'
        console.log(state)
        ui.misc.robotState.textContent = state
        if (connected) {
            // On connect hide the connect popup
            connectionDialog.close()
        } else {
            // On disconnect show the connect popup
            connectionDialog.showModal()
        }
    }, true)
}

openPage('default')

NetworkTables.addKeyListener('/SmartDashboard/misc/limelight/currentLimelight', (key, value) => {
    var cameraUrl = `http://limelight-${value}.local:5800`
    setupCamera(cameraUrl);
})

NetworkTables.addKeyListener('/SmartDashboard/misc/limelight/currentCamera', (key, value) => {
    if (value == 'limelight') {
        rotateCamera(0)
    } else if (value == 'usb') {
        rotateCamera(0)
    } else {
        throw new TypeError("Invalid current camera value.")
    }
})

function reloadCamera () {
    const content = ui.misc.camera.container.innerHTML
    ui.misc.camera.container.innerHTML = content
}

function rotateCamera (deg) {
    ui.misc.camera.cameraImg.setAttribute('style', `transform: rotate(${deg}deg)`)
}

function setupCamera (url) {
    ui.misc.camera.cameraImg.setAttribute('src', url)
    ui.misc.camera.cameraImg.setAttribute('class', 'camera')
    ui.misc.camera.cameraImg.addEventListener('onerror', () => {
        reloadCamera()
    })
}
