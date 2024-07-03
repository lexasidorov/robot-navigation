;

function createActions() {
	let actions = {};

	let btns = document.getElementsByClassName("action_button");
	for (let i = 0; i < btns.length; i++) {
		$(btns[i]).on("click", "", function() {
			getParam(this);
		});
	}

	let mv_btns = document.getElementsByClassName("move_button");
	for (let i = 0; i < mv_btns.length; i++) {
		let t;
		clearInterval();
		$(mv_btns[i]).on("mouseup", "", function() {
			clearInterval();
			t = setInterval(sendMvCmd(this), 1000);
		});
//		$(mv_btns[i]).on("mouseout", "", function() {
//			clearInterval();
//			t = setInterval(function() {sendMvCmd(this, 0);}, 1000);
//		});
		$(mv_btns[i]).on("mousedown", "", function() {
			t = setInterval(sendMvCmd(this), 1000);
		});
	}
	return actions;
}

function sendMvCmd(e) {
	let param_server_name = e.name;
	console.log("sendMvCmd = " + param_server_name); //param_server_name is empty wtf?
	let val = ((e.value == '1') ? '0' : '1')
//	if (forceVal == '0') {
//	    val = '0';
//	}
	$.post("/"+param_server_name+"/"+val, {param_server_name: val}, function(resp) {
		console.log(resp);
		e.value = parseInt(resp['state'][param_server_name]);
		let actionIsActive = (e.value == '1');
		if (actionIsActive) {
			yandex_translate(e);
		} else {
			yandex_translate_back(e);
		}
		// TODO: make a state checking and map it to button values
	}).fail(function(err) {
		console.error(err);
	});
}

function yandex_translate(e) {
	let icon = e.id;
	let action = e.name;
	let mv_btns = document.getElementsByClassName("move_button");
	e.style.background = "red";
	e.textContent = "x";
	for (let btn of mv_btns) {
		if (btn.name != action) {
			btn.disabled = true;
		}
	}
}

function yandex_translate_back(e) {
	let icon = e.id;
	let mv_btns = document.getElementsByClassName("move_button");
	e.style.background = "";
	e.textContent = icon;
	for (let btn of mv_btns) {
		btn.disabled = false;
	}
}

function getParam(e) {
	let param_server_name = e.name;
	let action_text = e.dataset.actionText;
	let cancel_action_text = e.dataset.actionCancelText;
	let val = ((e.value == '1') ? '0' : '1')
	console.log("getParam: " + param_server_name + " |  val: " + val);
	console.log("/"+param_server_name+"/"+val)
	$.post("/"+param_server_name+"/"+val, {param_server_name: val}, function(resp) {
		console.log(resp);
		e.value = parseInt(resp['state'][param_server_name]);
		e.textContent = ((e.value == '1') ? cancel_action_text : action_text)
		e.style.background = ((e.value == '1') ? "red" : "")
		// TODO: make a state checking and map it to button values
	}).fail(function(err) {
		console.error(err);
	});
}

$(document).ready(function () {
	createActions();
});