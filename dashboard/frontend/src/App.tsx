import './App.css';
import 'bootstrap/dist/css/bootstrap.min.css';
import NavBar from './navbar';
import Home from './pages/home';

const backend_path = "http://127.0.0.1:5000";

function App() {
  let component
  switch (window.location.pathname) {

      default:
        component = <Home />
  }


  return (
    <div className="App">
      <NavBar />
      <div className="container">{component}</div>
    </div>
  );
}

export default App;
