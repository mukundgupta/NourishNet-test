from flask import Flask, render_template
import main

app = Flask(__name__)

@app.route('/')
def home():
    
    return render_template("index.html")

@app.route('/page')
def map_page():
    c = main.main()
    return render_template("page.html",solution=c)


if __name__ == '__main__':
    app.run(debug=True)

