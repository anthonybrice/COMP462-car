// read-hex.js

"use strict"

const fs = require("fs")

var data = fs.readFileSync("left", { encoding: "utf8" })

data = data.replace(/(\r)/gm,"")

data = data.split("\n")
data.shift()
data.pop()
data.pop()

data = data.map(line => line.substring(11, line.length - 2))

data = data.reduce((p, q) => p + q, "")

data = data.match(/.{1,4}/g)

data = data.map(p => parseInt(p, 16))

console.log(data)
