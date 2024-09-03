"""
\file       server.py
\brief      Code to run a local server on computer and receive POST requests
            Requires finding local IP (run 'ifconfig | grep inet')
            Save the inet IP that is NOT localhost (127.0.0.1)

\authors    Corbin Warmbier | corbinwarmbier@gmail.com
\date       Initial: 07/13/24  |  Last: 07/13/24
"""

""" [Imports] """
from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import os
from sys import argv
import numpy as np
import math
import json

"""
Server Handling Class
Provides code for POST capabilities
"""
class S(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])  # Gets the size of data
        post_data = self.rfile.read(content_length)  # Gets the data itself
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                str(self.path), str(self.headers), post_data.decode('utf-8'))
        data = json.loads(post_data)
        gen_file_out(data["scan_data"], data["distance"])
        self._set_response()  # Send received response
        self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))

def run(server_class=HTTPServer, handler_class=S, port=8069):
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    print(server_address)
    httpd = server_class(server_address, handler_class)
    logging.info('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    logging.info('Stopping httpd...\n')

def gen_file_out(data, car_distance):
    data_json = {
        "scan_data": data,
        "distance": car_distance
    }

    file_path = 'lidar_scans.json'
    
    if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
        # If the file exists and is not empty, load the existing content
        with open(file_path, 'r', encoding='utf-8') as fp:
            existing_data = json.load(fp)
        
        # Append the new JSON object to the existing list
        existing_data.append(data_json)
        
    else:
        # If the file does not exist or is empty, create a new list with the JSON object
        existing_data = [data_json]
    
    # Write the updated list back to the file
    with open(file_path, 'w', encoding='utf-8') as fp:
        json.dump(existing_data, fp, indent=4)

# ========================================= #
#          === [Main Function] ===          #
# ========================================= #
if __name__ == '__main__':
    open('lidar_scans.json', 'w').close()  # Clear data.out file
    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()