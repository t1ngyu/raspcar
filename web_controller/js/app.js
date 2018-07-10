
/* Point */
class Point {
	constructor(x, y=0) {
		if (x instanceof Object) {
			this.x = x.x;
			this.y = x.y;
		} else {
		this.x = x;
		this.y = y;
		}
	}

	norm() {
		return Math.sqrt(this.x*this.x + this.y*this.y);
	}

	normalize() {
		var m = this.norm();
        if (m == 0) {
            return new Point(0, 0);
        }
		return new Point(this.x / m, this.y / m);
	}

	sub(p) {
		return new Point(this.x - p.x, this.y - p.y);
	}

	mul(n) {
		return new Point(this.x * n, this.y * n);
	}

	distance(p) {
		return this.sub(p).norm();
	}

	toString(n) {
        if (n != null) {
            return '(' + this.x.toFixed(n) + ',' + this.y.toFixed(n) + ')';
        }
		return '(' + this.x + ',' + this.y + ')';
	}

	clone() {
		return new Point(this.x, this.y);
	}
}

/* JoyStick */
class JoyStick {
	constructor(canvas) {
		this.canvas = canvas;
		this.activateSize = 2;
		this.actived = false;
		this.output = new Point(0, 0);
		this._basePanelRadius = 80;
		this._indicatorRadius = 30;
		this._fixedPosition = new Point(0, 0);
		this._basePanel = new fabric.Circle({
			radius: this._basePanelRadius,
			originX: 'center',
			originY: 'center',
			stroke: '#AAAAAA90',
			strokeWidth: 1,
			fill: '#AAAAAA50',
			left: 0,
			top: 0,
		});
		this._indicator = new fabric.Circle({
			radius: this._indicatorRadius,
			originX: 'center',
			originY: 'center',
			stroke: '#AAAAAA90',
			strokeWidth: 7,
			fill: '#666666F0',
			left: 0,
			top: 0,
		});
		this.bindingBox = new fabric.Rect({
			width: (this._basePanelRadius + this._indicatorRadius + 4)*2,
			height: (this._basePanelRadius + this._indicatorRadius + 4)*2,
			originX: 'center',
			originY: 'center',
			fill: "#ff000000",
			left: 0,
			top: 0,
		});
		this._group = new fabric.Group([this._basePanel, this._indicator, this.bindingBox], {
			originX: 'center',
			originY: 'center',
			selectable: false,
			left: 0,
			top: 0,
		});
		this.canvas.add(this._group);
	}

	get position() {
		return new Point(this._group.left, this._group.top);
	}

	onTouchDown(event) {
		if (this.actived)
			return false;
		var pos = event.pointer;
		if (!this.inActiveRange(pos))
			return false;
		this.activate(pos);
		this.canvas.renderAll();
		return true;
	}

	onTouchMove(event) {
		if (!this.actived) {
			return false;
		}
		var pos = new Point(event.pointer);
		var delta = pos.sub(this.position);
		var distance = delta.norm();
		var angle = Math.atan2(delta.y, delta.x);
		if (distance > this._basePanelRadius) {
			distance = this._basePanelRadius;
		}

		var indicatorPos = new Point(0, 0);
		if (distance != 0) {
			indicatorPos = new Point(distance * Math.cos(angle), distance * Math.sin(angle));
		}
		this._indicator.left = indicatorPos.x;
		this._indicator.top = indicatorPos.y;
		this._group.set('dirty',true);
		this.canvas.renderAll();
		this.output = indicatorPos.mul(1 / this._basePanelRadius);
		return true;
	}

	onTouchUp(event) {
		if (!this.actived)
			return false;
		this.deactivate();
		this.canvas.renderAll();
		return true;
	}

	setOriginalPosition(x, y) {
		this._fixedPosition.x = x;
		this._fixedPosition.y = y;
		this._group.left = x;
		this._group.top = y;
	}

	inActiveRange(pos) {
		if (this.position.distance(pos) < 2 * this._basePanelRadius) {
			return true;
		}
		return false;
	}

	activate(pos) {
		this.actived = true;
		this._group.left = pos.x;
		this._group.top = pos.y;
	}

	deactivate() {
		this.actived = false;
		this._group.left = this._fixedPosition.x;
		this._group.top = this._fixedPosition.y;
		this._indicator.left = 0;
		this._indicator.top = 0;
		this._group.set('dirty',true);
	}
}

/* websocket */
class WS {
    constructor(host) {
        this.host = host;
        this.websocket = null;
        this.onopen = this._log;
        this.onclose = this._log;
        this.onmessage = this._log;
        this.onerror = this._log;
    }

    init() {
        this.websocket = new WebSocket(this.host);
        this.websocket.onopen = this.onopen;
        this.websocket.onclose = this.onclose;
        this.websocket.onerror = this.onerror;
        this.websocket.onmessage = this.onmessage;
    }

    _log(event) {
        var text = null;
        if (event.type == 'message') {
            text = 'MESSAGE: ' + event.data;
        } else {
            text = event.type;
        }
        console.log(text);
    }

    send(message) {
        this.websocket.send(message);
    }
}

