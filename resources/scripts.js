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
    var ctx = document.grid_ctx;
    if (document.last_grid.length == document.grid.length) {
        var grids_equal = true;
        for (var i = 0; i < document.grid.length; i++) {
            if (document.last_grid[i] != document.grid[i]) {
                grids_equal = false;
                break;
            }
        }
        if (grids_equal) {
            return;
        }
    }

    console.log("update grid");

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
        var a = document.agents[i];
        if (a.kind != "Vehicle" || !a.on_map) {
            continue;
        }
        ctx.translate(a.x * scale, a.y * scale)
        ctx.rotate(-a.theta);
        ctx.fillRect(0, 0, a.width * scale, a.length * scale);
        ctx.fillStyle = "#0088ff";
        ctx.fillRect(0, 0, scale, scale);
        ctx.strokeRect(0, 0, a.width * scale, a.length * scale);
        // ctx.rotate(-Math.Pi / 3);
        ctx.font = "30px Arial";
        ctx.textBaseline = "top";
        ctx.fillText("" + a.health, 5, 5);
        ctx.fillStyle = "#00ffff";
        ctx.setTransform(1, 0, 0, 1, 0, 0); // reset transform to identity
    }

    ctx.fillStyle = "#ffff00";
    for (var i = 0; i < document.agents.length; i++) {
        var a = document.agents[i];
        if (a.kind != "Pedestrian" || !a.on_map) {
            continue;
        }
        ctx.translate(a.x * scale, a.y * scale)
        ctx.rotate(-a.theta);
        ctx.fillRect(0, 0, a.width * scale, a.length * scale);
        ctx.fillStyle = "#ee6600";
        ctx.fillRect(0, 0, a.width * scale * 2 / 3, a.length * scale * 2 / 3);
        ctx.fillStyle = "#ffff00";
        ctx.setTransform(1, 0, 0, 1, 0, 0); // reset transform to identity
    }
}

function update_stats() {
    document.getElementById("dead").innerHTML = "Deaths: <b>" + document.stats.dead + "</b>";
    document.getElementById("collisions").innerHTML = "Collisions: <b>" + document.stats.collisions + "</b>";
}

function update() {
    var r = new XMLHttpRequest();
    r.open("GET", "/update", true);
    r.onreadystatechange = function () {
        if (r.readyState != 4 || r.status != 200) {
            if (r.readyState == 4) {
                setTimeout(update, 100);
            }
            return;
        }
        var results = JSON.parse(r.responseText);
        document.agents = results.agents;
        document.grid = results.map;
        document.stats = results.stats;

        document.grid = results.map.grid;
        document.buildings = results.map.buildings;
        document.grid_width = results.map.grid_width;
        document.grid_height = document.grid.length / document.grid_width;
        update_grid();
        update_stats();

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
