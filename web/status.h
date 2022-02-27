#pragma once
const char* static_web_status_page= R"~~~~(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Status</title>
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script>
    <script>
        var start = new Date;
        function populate(){
            var host = window.location.protocol + "//" + window.location.host;
            console.log(host);
            $.ajax({
                url: host+"/json/status"
            }).then(function(data) {
                var obj = jQuery.parseJSON(data);
                var board1_msgs = obj.status.hw1.board.msgs
                var board2_msgs = obj.status.hw2.board.msgs

                var velo1_rej_rate = obj.status.hw2.velodyne_1.rejection_rate;
                var velo2_rej_rate = obj.status.hw2.velodyne_2.rejection_rate;

                var velo1_data_rate = obj.status.hw2.velodyne_1.data_rate;
                var velo2_data_rate = obj.status.hw2.velodyne_2.data_rate;

                var velo1_jitter = obj.status.hw2.velodyne_1.jitter;
                var velo2_jitter = obj.status.hw2.velodyne_2.jitter;

                var livox_rej_rate = obj.status.hw1.livox_1.rejection_rate;
                var livox_data_rate = obj.status.hw1.livox_1.data_rate;

                var board_1_tt = obj.status.hw1.board.tt;
                var board_2_tt = obj.status.hw2.board.tt;

                var hw1hw2_jitter = obj.status.hw1hw2.jitter;
                var all_ok = true;

                $('.board_1_msgs').text("board1_msgs: "+board1_msgs);
                if (board1_msgs < 450){ $('.board_1_msgs').css({'color': 'red'});all_ok = false;}else{ $('.board_1_msgs').css({'color': 'green'});}

                $('.board_2_msgs').text("board2_msgs: "+board2_msgs);
                if (board2_msgs < 450){ $('.board_2_msgs').css({'color': 'red'});all_ok = false;}else{ $('.board_2_msgs').css({'color': 'green'});}

                $('.velo1_rej_rate').text("velo1_rej_rate: "+velo1_rej_rate);
                if (velo1_rej_rate <5 && velo1_rej_rate >= 0){ $('.velo1_rej_rate').css({'color': 'green'});}else{ $('.velo1_rej_rate').css({'color': 'red'});all_ok = false;}

                $('.velo2_rej_rate').text("velo2_rej_rate: "+velo2_rej_rate);
                if (velo2_rej_rate <5 && velo2_rej_rate >= 0){ $('.velo2_rej_rate').css({'color': 'green'});}else{ $('.velo2_rej_rate').css({'color': 'red'});all_ok = false;}

                $('.velo1_data_rate').text("velo1_data_rate: "+velo1_data_rate);
                if (velo1_data_rate> 150000){ $('.velo1_data_rate').css({'color': 'green'});}else{ $('.velo1_data_rate').css({'color': 'red'});all_ok = false;}

                $('.velo2_data_rate').text("velo1_data_rate: "+velo2_data_rate);
                if (velo2_data_rate> 150000){ $('.velo2_data_rate').css({'color': 'green'});}else{ $('.velo2_data_rate').css({'color': 'red'});all_ok = false;}

                $('.velo1_jitter').text("velo1_jitter: "+velo1_jitter);
                if (velo1_jitter< 0.001 || velo1_jitter>-0.001  ){ $('.velo1_jitter').css({'color': 'green'});}else{ $('.velo1_jitter').css({'color': 'red'});all_ok = false;}

                $('.velo2_jitter').text("velo2_jitter: "+velo2_jitter);
                if (velo2_jitter< 0.001 || velo2_jitter>-0.001){ $('.velo2_jitter').css({'color': 'green'});}else{ $('.velo2_jitter').css({'color': 'red'});all_ok = false;}


                $('.livox_rej_rate').text("livox_rej_rate: "+livox_rej_rate);
                if (livox_rej_rate <5 && livox_rej_rate >= 0){ $('.livox_rej_rate').css({'color': 'green'});}else{ $('.livox_rej_rate').css({'color': 'red'});all_ok = false;}

                $('.livox_data_rate').text("livox_data_rate: "+livox_data_rate);
                if (livox_data_rate > 90000 ){ $('.livox_data_rate').css({'color': 'green'});}else{ $('.livox_data_rate').css({'color': 'red'});all_ok = false;}

                $('.hw1hw2_jitter').text("hw1hw2_jitter: "+hw1hw2_jitter);
                if (hw1hw2_jitter >-0.001 && hw1hw2_jitter <0.001){ $('.hw1hw2_jitter').css({'color': 'green'});}else{ $('.hw1hw2_jitter').css({'color': 'red'});all_ok = false;}

                $('.board_1_tt').text(board_1_tt);
                $('.board_2_tt').text(board_2_tt);


                if (all_ok){
                    $('.system_emot').html("&#128512;"); // :)
                }else{
                    $('.system_emot').html("&#129314;") // :(
                }
            });
        };
        setInterval(function() {populate();}, 1000);

        $(document).ready(function() {populate();});

        function trg_button_fun() {
          var host = window.location.protocol + "//" + window.location.host;
          var data = document.getElementById('scan_name').value
          console.log("calling "+host+"/trig/scan?"+data);

          $.ajax({
                url: host+"/trig/scan?"+data
          })
        }

       function trg_button_reset(board) {
          var host = window.location.protocol + "//" + window.location.host;
          var data = document.getElementById('scan_name').value
          console.log("calling "+host+"/trig/resetBoard?"+board);

          $.ajax({
                url: host+"/trig/resetBoard?"+board
          })
        }

    </script>

</head>

<body>
<label for="fname">Scan Name:</label>
<input type="/home/robot/test" id="scan_name" name="scan_name" value="/tmp/test_fn/"><br><br>
<button type="button" id="btn_trigg" onclick="trg_button_fun()" style="display:inline;" >Trigger Scan</button>

<div>
    <h1> System Health <div class="system_emot" style="display:inline;"> &#129314;</div></h1>
    <h2> Board 1 :</h2>

    <dl>
        <dt>Board</dt>
        <dd><p class="board_1_msgs"> </p></dd>

        <dt>Livox</dt>
        <dd><p class="livox_rej_rate"> </p></dd>
        <dd><p class="livox_data_rate"> </p></dd>


    </dl>
    <h2> Board 2 :</h2>
    <dl>
        <dt>Board</dt>
        <dd><p class="board_2_msgs"></p></dd>
        <dt>Velodyne 1</dt>
        <dd><p class="velo1_rej_rate"> </p></dd>
        <dd><p class="velo1_data_rate"> </p></dd>
        <dd><p class="velo1_jitter"></p></dd>

        <dt>Velodyne 2</dt>
        <dd><p class="velo2_rej_rate"></p></dd>
        <dd><p class="velo2_data_rate"></p></dd>
        <dd><p class="velo2_jitter"></p></dd>
    </dl>
</div>

<h2> Board 1/2:</h2>
<dl>
    <dt>Board</dt>
    <dd><p class="hw1hw2_jitter"></p></dd>
</dl>
<h2> NMEA / PPS status :</h2>
<table style="width:100%">
  <tr>
    <th>Board1</th>
    <th>Board2</th>
  </tr>
  <tr>
    <td> <button type="button" onclick="trg_button_reset(1)" style="display:inline;" >Reset Board</button></td>
    <td> <button type="button" onclick="trg_button_reset(2)" style="display:inline;" >Reset Board</button></td>
  </tr>
  <tr>
    <td><code><pre><p class="board_1_tt"></p></pre></code></td>
    <td><code><pre><p class="board_2_tt"></p></pre></code></td>
  </tr>
</table>

</body>
</html>
)~~~~";