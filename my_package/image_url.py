import http.server
import socketserver

PORT = 5000
DIRECTORY = "/home/hiratalab/catkin_ws/src/shu_task_planning/my_package/public/"


class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)


handler = CustomHTTPRequestHandler

with socketserver.TCPServer(("", PORT), handler) as httpd:
    print(f"Serving at port {PORT}")
    httpd.serve_forever()
