var scale = 10;

function update_color(ctx, grid, last_grid, value, color) {
    ctx.fillStyle = color;
    var idx = 0;
    for (var y = 0; y < document.grid_height; y++) {
        for (var x = 0; x < document.grid_width; x++, idx++) {
            if ((grid[idx] & value) == value) {//} && grid[idx] != last_grid[idx]) {
                ctx.fillRect(x * scale, y * scale, scale - 1, scale - 1);
            }
        }
    }
}

function update_grid() {
    console.log("update grid");
    var ctx = document.grid_ctx;
    if (document.last_grid == document.grid) {
        return;
    }

    ctx.fillStyle = "#888888";
    ctx.fillRect(0, 0, document.grid_canvas.width, document.grid_canvas.height);

    update_color(ctx, document.grid, document.last_grid, 0, '#000000');
    update_color(ctx, document.grid, document.last_grid, 1, '#6600ff');
    update_color(ctx, document.grid, document.last_grid, 2, '#8866cc');
    // update_color(ctx, document.grid, document.last_grid, 3, '#ffff00');
    update_color(ctx, document.grid, document.last_grid, 4, '#0000ff');
    update_color(ctx, document.grid, document.last_grid, 8, '#ffffff');

    document.last_grid = Array.from(document.grid);

    // buildings
    ctx.font = "30px Arial";
    ctx.textBaseline = "top";
    for (var i = 0; i < document.buildings.length; i++) {
        var building = document.buildings[i];
        ctx.fillText("B", building[0] * scale, building[1] * scale);
    }
}

function render() {
    var canvas = document.getElementById("map_canvas");
    var ctx = canvas.getContext("2d");

    ctx.drawImage(document.grid_canvas, 0, 0);

    ctx.fillStyle = "#00ffff";
    for (var i = 0; i < document.agents.length; i++) {
        var p = document.agents[i];
        if (p.kind != "Vehicle" || !p.on_map) {
            continue;
        }
        ctx.translate(p.x * scale, p.y * scale)
        ctx.rotate(-p.theta);
        ctx.fillRect(0, 0, p.width * scale, p.length * scale);
        ctx.fillStyle = "#0088ff";
        ctx.fillRect(0, 0, scale, scale);
        ctx.strokeRect(0, 0, p.width * scale, p.length * scale);
        ctx.fillStyle = "#00ffff";
        ctx.setTransform(1, 0, 0, 1, 0, 0); // reset transform to identity
    }

    ctx.fillStyle = "#ffff00";
    for (var i = 0; i < document.agents.length; i++) {
        var p = document.agents[i];
        if (p.kind != "Pedestrian" || !p.on_map) {
            continue;
        }
        ctx.translate(p.x * scale, p.y * scale)
        ctx.rotate(-p.theta);
        ctx.fillRect(0, 0, p.width * scale, p.length * scale);
        ctx.fillStyle = "#ee6600";
        ctx.fillRect(0, 0, p.width * scale * 2 / 3, p.length * scale * 2 / 3);
        ctx.fillStyle = "#ffff00";
        ctx.setTransform(1, 0, 0, 1, 0, 0); // reset transform to identity
    }
}

function update() {
    var r = new XMLHttpRequest();
    r.open("GET", "/agents", true);
    r.onreadystatechange = function () {
        if (r.readyState != 4 || r.status != 200) {
            if (r.readyState == 4) {
                setTimeout(update, 200);
            }
            return;
        }
        document.agents = JSON.parse(r.responseText);
        requestAnimationFrame(render);
        setTimeout(update, 200);
    };
    r.send();
}

function init() {
    var canvas = document.getElementById("map_canvas");
    canvas.width = document.grid_width * scale;
    canvas.height = document.grid_height * scale;

    document.grid_canvas = document.createElement("canvas");
    document.grid_canvas.width = canvas.width;
    document.grid_canvas.height = canvas.height;
    document.grid_ctx = document.grid_canvas.getContext("2d");
    document.last_grid = [];
    for (var i = 0; i < document.grid.length; i++) document.last_grid[i] = -1;

    document.agents = {pedestrians: [], vehicles: []};

    update_grid();
    update();
}

document.render_map = render;
window.onload = init;
