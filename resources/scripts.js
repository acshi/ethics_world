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
    update_color(ctx, document.grid, document.last_grid, 1, '#ff0000');
    update_color(ctx, document.grid, document.last_grid, 2, '#00ff00');
    update_color(ctx, document.grid, document.last_grid, 3, '#ffff00');
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
}

function update() {
    requestAnimationFrame(render);
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

    update_grid();
    update();
    setInterval(update, 5000);
}

window.onload = init;
