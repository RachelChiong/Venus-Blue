import React, { useState, useEffect } from 'react';
import '../App.css';
import 'bootstrap/dist/css/bootstrap.min.css';
import { Col, Container, ProgressBar, Row } from 'react-bootstrap';
import Highcharts from 'highcharts';
import HighchartsReact from 'highcharts-react-official';
import HighchartsMore from 'highcharts/highcharts-more';
import axios from 'axios';

// Specify path to backend API
const backend_path = "http://127.0.0.1:5001";
// Initialize Highcharts modules
HighchartsMore(Highcharts);

function Home() {

    interface Location {
        "x": number,
        "y": number
    };

    interface FootpedalPacket {
        "x": number,
        "y": number,
        "z": number
    };

    interface TelemetryPacket {
        "vx": number,
        "vy": number,
        "linear": number,
        "angular": number,
        "xLocation": number,
        "yLocation": number
    };

    interface LocalisationPacket {
        "x": number,
        "y": number
    };

    const [footpedal, setFootpedal] = useState<FootpedalPacket>({x: 0, y: 0, z: 0});
    const [telemetry, setTelemetry] = useState<TelemetryPacket>({vx: 0, vy: 0, linear: 0, angular: 0, xLocation: 0, yLocation: 0});
    const [localisation, setLocalisation] = useState<LocalisationPacket>({x: 0, y: 0});
    const [telemetryGraph, setTelemetryGraph] = useState<TelemetryPacket>({vx: 0, vy: 0, linear: 0, angular:0, xLocation: 0, yLocation: 0});
    const [isLoading, setIsLoading] = useState(Boolean);

    useEffect(() => {
        const fetchData = async () => {
            setIsLoading(true);

            try {
                const response = await axios.get(backend_path + "/footpedal");
                setFootpedal(response.data);
                console.log(response.data);

                const responseLocalisation = await axios.get(backend_path + "/localisation");
                setLocalisation(responseLocalisation.data);
                console.log(responseLocalisation.data);

                const responseTelemetry = await axios.get(backend_path + "/telemetry");
                setTelemetry(responseTelemetry.data);
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
        x: Math.abs(telemetry.xLocation) > 2 ? 200 : Math.abs(telemetry.xLocation) * 100,
        y: Math.abs(telemetry.yLocation) > 2 ? 200 : Math.abs(telemetry.yLocation) * 100,
        marker: {
            symbol: 'url(https://avatars.githubusercontent.com/u/2200113?v=4)',
            width: 60,
            height: 60
        }
    }];

    let telemetryData = [
        {x: telemetryGraph.angular, y: Math.abs(telemetryGraph.linear)}
    ]

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
            data: data,
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
            <div className="Home home-item" style={{width: "50%"}}>
                <HighchartsReact highcharts={Highcharts} options={options}/>
            </div>

            <div className='home-item' style={{width: "40%"}}>
            <br/>
                <h3>Footpedal Values</h3>

                <Container fluid>
                    <Row className='align-middle'>
                        <Col sm={4}><b>x: </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                            <ProgressBar id="Footpedal-X" now={(footpedal.x % 128)/127 * 100} label={footpedal.x}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col sm={4}><b>y: </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                            <ProgressBar id="Footpedal-Y" now={(footpedal.y % 64)/63 * 100} label={footpedal.y}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col sm={4}><b>z: </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                            <ProgressBar id="Footpedal-Z" now={footpedal.z/128 * 100} label={footpedal.z} />
                        </Col>
                    </Row>
                </Container>

                <br/>
                <h3>Telemetry Data</h3>

                <Container fluid>
                    <Row className='align-middle'>
                        <Col sm={4}><b>vx (m/s): </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                        <ProgressBar id="telemetry-vx" now={((telemetry.vx/0.2) * 50) + 50} label={telemetry.vx}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col sm={4}><b>vy (m/s): </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                        <ProgressBar id="telemetry-vy" now={((telemetry.vy/0.2) * 50) + 50} label={telemetry.vx}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col sm={4}><b>linear (m/s): </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                        <ProgressBar id="telemetry-lin" now={((telemetry.linear/0.2) * 50) + 50} label={telemetry.linear} />
                        </Col>
                    </Row>
                    <Row>
                        <Col sm={4}><b>angular (rad): </b></Col>
                        <Col sm={8} style={{marginTop: "2%"}}>
                        <ProgressBar id="telemetry-lin" now={((telemetry.angular) * 50) + 50} label={telemetry.angular} />
                        </Col>
                    </Row>
                </Container>
                <br/>
                <div>
                    <p>Movement Mode: {telemetry.linear < 0 ? "Reverse" : (telemetry.linear === 0 ? "Stationary" : "Forward")}</p>
                </div>
            </div>
        </div>
    );
}

export default Home;
