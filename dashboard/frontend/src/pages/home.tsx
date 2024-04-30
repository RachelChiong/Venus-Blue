import React, { useState, useEffect } from 'react';
import '../App.css';
import 'bootstrap/dist/css/bootstrap.min.css';

import Button from 'react-bootstrap/Button';
import ButtonGroup from 'react-bootstrap/ButtonGroup';
import { Form } from 'react-bootstrap';

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

    const [distance, setDistance] = useState("");
    const [kalman, setKalman] = useState<Location[]>([{ x: 0, y: 0 }]);
    const [multiLat, setMultiLat] = useState<Location[]>([{ x: 0, y: 0 }]);
    const [usonicLocation, setUsonicLocation] = useState("4011-B");
    const [usonicResp, setUsonicResp] = useState<Usonic_Resp>({ name: "", coordinates: { x: 200, y: 0 }, response: "" });
    const [isLoading, setIsLoading] = useState(Boolean);

    const beaconIDs = ['4011-A', '4011-B', '4011-C', '4011-D', '4011-E', '4011-F', '4011-G', '4011-H'];


    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);
            try {
                const response = await axios.get(backend_path + "/kalman");
                setKalman(response.data);
            } catch (error) {
                ;
            } finally {
                setIsLoading(false);
            }
        };

        const intervalId = setInterval(fetchData, 1000);

        // Cleanup function to clear the interval on unmount
        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);
            try {
                const response_ultrasonic = await axios.get(backend_path + "/ultrasonic");
                setDistance(response_ultrasonic.data.distance)
            } catch (error) {
                ;
            } finally {
                setIsLoading(false);
            }
        };

        const intervalId = setInterval(fetchData, 1000);

        // Cleanup function to clear the interval on unmount
        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);
            try {
                const response_multi = await axios.get(backend_path + "/multilaterations");
                setMultiLat(response_multi.data.multilateration);
            } catch (error) {
                ;
            } finally {
                setIsLoading(false);
            }
        };

        const intervalId = setInterval(fetchData, 1000);

        // Cleanup function to clear the interval on unmount
        return () => clearInterval(intervalId);
    }, []);


    const set_ultrasonic_location = async (beacon_name: string) => {
        const data = {
            name: beacon_name
        };
        try {
            const response = await axios.post(backend_path + "/beacon/ultrasonic", data); // Replace with your API endpoint
            console.log('POST request successful:', response.data);
            setUsonicResp(response.data);

            // Handle successful response (e.g., display a success message)
        } catch (error) {
            console.error('Error sending POST request:', error);

            // Handle errors (e.g., display an error message)
        }
    }


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

                <p>Foot pedal Response: ({usonicResp.coordinates.x * 100}, {usonicResp.coordinates.y * 100})</p>
            </div>
        </div>
    );
}

export default Home;
