CANのライブラリを書くにあたって気を付けることとして、   
・Initializeデータを除くバッファは1byteでも削れそうな部分があれば削る。     
・目標値が前回値と同じ場合は送らない。      

CANのIDは例えばYamaMDに関するものはyamamdの名前空間内に定義してあります。   