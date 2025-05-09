package frc.robot.web;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import frc.robot.Constants.PIDConfig;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class PIDTunerServer {
  private final int port;
  private HttpServer server;
  private final AtomicBoolean hasUpdates = new AtomicBoolean(false);
  
  // PID configurations
  private PIDConfig drivePID = new PIDConfig(
    Constants.DEFAULT_DRIVE_PID.kP,
    Constants.DEFAULT_DRIVE_PID.kI,
    Constants.DEFAULT_DRIVE_PID.kD,
    Constants.DEFAULT_DRIVE_PID.kF
  );
  
  private PIDConfig turnPID = new PIDConfig(
    Constants.DEFAULT_TURN_PID.kP,
    Constants.DEFAULT_TURN_PID.kI,
    Constants.DEFAULT_TURN_PID.kD,
    Constants.DEFAULT_TURN_PID.kF
  );
  
  public PIDTunerServer(int port) {
    this.port = port;
  }
  
  public void start() {
    try {
      server = HttpServer.create(new InetSocketAddress(port), 0);
      server.createContext("/", new RootHandler());
      server.createContext("/update", new UpdateHandler());
      server.setExecutor(null);
      server.start();
      System.out.println("PID Tuner server started on port " + port);
    } catch (IOException e) {
      System.err.println("Failed to start PID Tuner server: " + e.getMessage());
    }
  }
  
  public void stop() {
    if (server != null) {
      server.stop(0);
      System.out.println("PID Tuner server stopped");
    }
  }
  
  public boolean hasUpdates() {
    return hasUpdates.getAndSet(false);
  }
  
  public PIDConfig getDrivePID() {
    return drivePID;
  }
  
  public PIDConfig getTurnPID() {
    return turnPID;
  }
  
  private class RootHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      String html = generateHtml();
      exchange.getResponseHeaders().set("Content-Type", "text/html");
      exchange.sendResponseHeaders(200, html.length());
      
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(html.getBytes(StandardCharsets.UTF_8));
      }
    }
    
    private String generateHtml() {
      return """
        <!DOCTYPE html>
        <html>
        <head>
          <title>FRC Swerve Drive PID Tuner</title>
          <meta name="viewport" content="width=device-width, initial-scale=1">
          <style>
            body {
              font-family: Arial, sans-serif;
              max-width: 800px;
              margin: 0 auto;
              padding: 20px;
            }
            h1 {
              color: #333;
              text-align: center;
            }
            .container {
              display: flex;
              flex-wrap: wrap;
              gap: 20px;
            }
            .pid-section {
              flex: 1;
              min-width: 300px;
              border: 1px solid #ddd;
              border-radius: 5px;
              padding: 15px;
              background-color: #f9f9f9;
            }
            .form-group {
              margin-bottom: 15px;
            }
            label {
              display: block;
              margin-bottom: 5px;
              font-weight: bold;
            }
            input[type="number"] {
              width: 100%;
              padding: 8px;
              border: 1px solid #ddd;
              border-radius: 4px;
            }
            button {
              background-color: #0066cc;
              color: white;
              border: none;
              padding: 10px 15px;
              border-radius: 4px;
              cursor: pointer;
              font-size: 16px;
              margin-top: 10px;
            }
            button:hover {
              background-color: #0052a3;
            }
            .status {
              margin-top: 20px;
              padding: 10px;
              border-radius: 4px;
              text-align: center;
            }
            .success {
              background-color: #d4edda;
              color: #155724;
            }
            .error {
              background-color: #f8d7da;
              color: #721c24;
            }
          </style>
        </head>
        <body>
          <h1>FRC Swerve Drive PID Tuner</h1>
          
          <div class="container">
            <div class="pid-section">
              <h2>Drive Motor PID</h2>
              <div class="form-group">
                <label for="drive-p">P Gain:</label>
                <input type="number" id="drive-p" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="drive-i">I Gain:</label>
                <input type="number" id="drive-i" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="drive-d">D Gain:</label>
                <input type="number" id="drive-d" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="drive-f">F Gain:</label>
                <input type="number" id="drive-f" step="0.001" value="%f">
              </div>
              <button onclick="updateDrivePID()">Update Drive PID</button>
            </div>
            
            <div class="pid-section">
              <h2>Turn Motor PID</h2>
              <div class="form-group">
                <label for="turn-p">P Gain:</label>
                <input type="number" id="turn-p" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="turn-i">I Gain:</label>
                <input type="number" id="turn-i" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="turn-d">D Gain:</label>
                <input type="number" id="turn-d" step="0.001" value="%f">
              </div>
              <div class="form-group">
                <label for="turn-f">F Gain:</label>
                <input type="number" id="turn-f" step="0.001" value="%f">
              </div>
              <button onclick="updateTurnPID()">Update Turn PID</button>
            </div>
          </div>
          
          <div id="status" class="status" style="display: none;"></div>
          
          <script>
            function updateDrivePID() {
              const p = document.getElementById('drive-p').value;
              const i = document.getElementById('drive-i').value;
              const d = document.getElementById('drive-d').value;
              const f = document.getElementById('drive-f').value;
              
              updatePID('drive', p, i, d, f);
            }
            
            function updateTurnPID() {
              const p = document.getElementById('turn-p').value;
              const i = document.getElementById('turn-i').value;
              const d = document.getElementById('turn-d').value;
              const f = document.getElementById('turn-f').value;
              
              updatePID('turn', p, i, d, f);
            }
            
            function updatePID(type, p, i, d, f) {
              const data = {
                type: type,
                p: parseFloat(p),
                i: parseFloat(i),
                d: parseFloat(d),
                f: parseFloat(f)
              };
              
              fetch('/update', {
                method: 'POST',
                headers: {
                  'Content-Type': 'application/json'
                },
                body: JSON.stringify(data)
              })
              .then(response => response.json())
              .then(data => {
                const statusElement = document.getElementById('status');
                statusElement.style.display = 'block';
                
                if (data.success) {
                  statusElement.className = 'status success';
                  statusElement.textContent = data.message;
                } else {
                  statusElement.className = 'status error';
                  statusElement.textContent = data.message || 'An error occurred';
                }
                
                setTimeout(() => {
                  statusElement.style.display = 'none';
                }, 3000);
              })
              .catch(error => {
                console.error('Error:', error);
                const statusElement = document.getElementById('status');
                statusElement.style.display = 'block';
                statusElement.className = 'status error';
                statusElement.textContent = 'Failed to update PID values';
                
                setTimeout(() => {
                  statusElement.style.display = 'none';
                }, 3000);
              });
            }
          </script>
        </body>
        </html>
      """.formatted(
        drivePID.kP, drivePID.kI, drivePID.kD, drivePID.kF,
        turnPID.kP, turnPID.kI, turnPID.kD, turnPID.kF
      );
    }
  }
  
  private class UpdateHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!"POST".equals(exchange.getRequestMethod())) {
        sendResponse(exchange, 405, "{\"success\":false,\"message\":\"Method not allowed\"}");
        return;
      }
      
      try {
        // Parse request body
        byte[] requestBody = exchange.getRequestBody().readAllBytes();
        String requestString = new String(requestBody, StandardCharsets.UTF_8);
        Map<String, Object> data = parseJson(requestString);
        
        String type = (String) data.get("type");
        double p = ((Number) data.get("p")).doubleValue();
        double i = ((Number) data.get("i")).doubleValue();
        double d = ((Number) data.get("d")).doubleValue();
        double f = ((Number) data.get("f")).doubleValue();
        
        // Update PID values
        if ("drive".equals(type)) {
          drivePID = new PIDConfig(p, i, d, f);
        } else if ("turn".equals(type)) {
          turnPID = new PIDConfig(p, i, d, f);
        } else {
          sendResponse(exchange, 400, "{\"success\":false,\"message\":\"Invalid PID type\"}");
          return;
        }
        
        // Set update flag
        hasUpdates.set(true);
        
        // Send success response
        sendResponse(exchange, 200, "{\"success\":true,\"message\":\"" + type + " PID values updated\"}");
      } catch (Exception e) {
        sendResponse(exchange, 400, "{\"success\":false,\"message\":\"" + e.getMessage() + "\"}");
      }
    }
    
    private void sendResponse(HttpExchange exchange, int statusCode, String response) throws IOException {
      exchange.getResponseHeaders().set("Content-Type", "application/json");
      exchange.sendResponseHeaders(statusCode, response.length());
      
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(response.getBytes(StandardCharsets.UTF_8));
      }
    }
    
    private Map<String, Object> parseJson(String json) {
      // Simple JSON parser for this example
      // In a real application, use a proper JSON library
      Map<String, Object> result = new HashMap<>();
      
      // Remove braces and split by commas
      json = json.trim();
      if (json.startsWith("{")) {
        json = json.substring(1);
      }
      if (json.endsWith("}")) {
        json = json.substring(0, json.length() - 1);
      }
      
      String[] pairs = json.split(",");
      for (String pair : pairs) {
        String[] keyValue = pair.split(":");
        if (keyValue.length == 2) {
          String key = keyValue[0].trim();
          key = key.replaceAll("\"", "");
          
          String value = keyValue[1].trim();
          if (value.startsWith("\"") && value.endsWith("\"")) {
            // String value
            value = value.substring(1, value.length() - 1);
            result.put(key, value);
          } else {
            // Numeric value
            try {
              double numValue = Double.parseDouble(value);
              result.put(key, numValue);
            } catch (NumberFormatException e) {
              result.put(key, value);
            }
          }
        }
      }
      
      return result;
    }
  }
}