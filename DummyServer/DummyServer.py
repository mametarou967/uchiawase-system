from flask import *
import sys

app = Flask( __name__ )

@app.route('/')
def index():
	id = request.args.get('id')
	if(id == '13045'):
		return jsonify({"result":"OK"})
	else:
		return jsonify({"result":"NG"})

def hello_world():
	return 'Hello World'
	
if __name__ == '__main__':
	app.run(host='0.0.0.0', port=5000,debug=True)

