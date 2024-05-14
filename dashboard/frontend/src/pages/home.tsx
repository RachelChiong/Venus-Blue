import React, { useState, useEffect } from 'react';
import '../App.css';
import 'bootstrap/dist/css/bootstrap.min.css';

import Button from 'react-bootstrap/Button';
import ButtonGroup from 'react-bootstrap/ButtonGroup';
import { Form, ProgressBar } from 'react-bootstrap';

import Highcharts from 'highcharts'
import HighchartsReact from 'highcharts-react-official'
import axios from 'axios';

const backend_path = "http://127.0.0.1:5001";


function Home() {
    interface Location {
        "x": number,
        "y": number
    };

    interface Usonic_Resp {
        "name": string,
        "coordinates": {
            "x": number,
            "y": number
        },

        "response": string
    };

    interface FootpedalPacket {
        "x": number,
        "y": number,
        "z": number
    };

    interface TelemetryPacket {
        "vx": number,
        "vy": number
    };

    const [distance, setDistance] = useState("");
    const [kalman, setKalman] = useState<Location[]>([{ x: 0, y: 0 }]);
    const [multiLat, setMultiLat] = useState<Location[]>([{ x: 0, y: 0 }]);
    const [usonicLocation, setUsonicLocation] = useState("4011-B");
    const [usonicResp, setUsonicResp] = useState<Usonic_Resp>({ name: "", coordinates: { x: 200, y: 0 }, response: "" });
    const [isLoading, setIsLoading] = useState(Boolean);

    const [footpedal, setFootpedal] = useState<FootpedalPacket>({x: 0, y: 0, z: 0});
    const [telemetry, setTelemetry] = useState<TelemetryPacket>({vx: 0, vy: 0});

    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);

            try {
                const response = await axios.get(backend_path + "/footpedal");
                console.log(response);
                setFootpedal(response.data);
                console.log(response.data);
            } catch (error) {
                ;
            } finally {
                setIsLoading(false);
            }
        };

        const intervalId = setInterval(fetchData, 100);

        // Cleanup function to clear the interval on unmount
        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);

            try {
                const response = await axios.get(backend_path + "/telemetry");
                console.log(response);
                setTelemetry(response.data);
                console.log(response.data);
            } catch (error) {
                ;
            } finally {
                setIsLoading(false);
            }
        };

        const intervalId = setInterval(fetchData, 100);

        // Cleanup function to clear the interval on unmount
        return () => clearInterval(intervalId);
    }, []);


    let data = [{
        x: 200,
        y: distance
    }];

    const options = {
        chart: {
            height: 600,
            width: 600
        },

        title: {
            text: 'Turtlebot Position'
        },

        series: [
        {
            type: 'scatter',
            name: "Localisation",
            data: multiLat
        }],

        xAxis: {
            min: 0,
            max: 200,
            gridLineWidth: 2,
            tickInterval: 50,
            title: {
                text: "x-Grid Position (cm)"
            }
        },

        yAxis: {
            min: 0,
            max: 200,
            gridLineWidth: 2,
            tickInterval: 50,
            title: {
                text: "y-Grid Position (cm)"
            }
        }
    }

    return (
        <div style={{ display: 'flex', justifyContent: 'center' }}>
            <div className="Home">
                <p> Turtlebot position: {!multiLat ? "Loading..." : multiLat[0].x}, {!multiLat ? "Loading..." : multiLat[0].y}</p>
                <HighchartsReact
                    highcharts={Highcharts}
                    options={options}
                />

                <h3>FootPedal Values</h3>
                <ProgressBar id="Footpedal-X" now={(footpedal.x % 128)/127 * 100} label={footpedal.x}/>
                <ProgressBar id="Footpedal-Y" now={(footpedal.y % 64)/63 * 100} label={footpedal.y}/>
                <ProgressBar id="Footpedal-Z" now={footpedal.z/128 * 100} label={footpedal.z} />

                <p>Foot pedal Response: (x: {footpedal.x}, y: {footpedal.y}, z: {footpedal.z})</p>

                <h3>Telemetry Values</h3>
                <p>Telemetry Response: (vx: {telemetry.vx}, vy: {telemetry.vy})</p>
            </div>
        </div>
    );
}

export default Home;
