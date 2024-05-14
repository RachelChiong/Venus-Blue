import React from 'react';
import Container from 'react-bootstrap/Container';
import Nav from 'react-bootstrap/Nav';
import Navbar from 'react-bootstrap/Navbar';
import axios from 'axios';

const backend_path = "http://127.0.0.1:5001";

export default function NavBar() {

    const fetchData = async () => {
        try {
            const response = await axios.get(backend_path);
            console.log("Data loaded successfully");
        } catch (error) {
            console.error("Error fetching data:", error);
        }
    };

  return (
    <Navbar expand="lg" className="bg-body-tertiary">
    <Container>
      <Navbar.Brand href="#home">PC-GUI</Navbar.Brand>
      <Navbar.Toggle aria-controls="basic-navbar-nav" />
      <Navbar.Collapse id="basic-navbar-nav">
        <Nav className="me-auto">
          <Nav.Link style={{color: "blue"}} href="#connect" onClick={() => fetchData()}>Connect</Nav.Link>
          <Nav.Link href="/">Home</Nav.Link>
        </Nav>
      </Navbar.Collapse>
    </Container>
  </Navbar>
  );
}
