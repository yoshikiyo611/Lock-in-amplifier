module FpgaTop (
   input  wire        clk,			 // 12MHz システムクロック
	input  wire        rst,			 // 外部リセット (Negative Logic)
	input  wire        in,         // 入力信号（同期対象）
	output reg         sq_out_0,   // 0° 同期矩形波
	output reg         sq_out_90,  // 90° 同期矩形波
	output reg         locked  	 // 位相ロック検出
);

    //============================================================
    // パラメータ・定数定義
    //============================================================
    localparam PHASE_WIDTH      = 32;
    localparam FRAC_WIDTH       = 8;

    // 位相・周波数関連
    localparam signed [24:0] INITIAL_INCR  = {17'd0,  	{FRAC_WIDTH{1'b0}}};
    localparam signed [24:0] FREQ_INCR_MIN = {17'd10,		{FRAC_WIDTH{1'b0}}};		// 30Hz
    localparam signed [24:0] FREQ_INCR_MAX = {17'd50332,	{FRAC_WIDTH{1'b0}}};		// 150kHz
    localparam [23:0]        PHASE_90_OFFSET = 24'h400000; // 90度相当
    localparam [15:0]        PHASE_OFFSET    = 16'h7FFF;   // 理想的な位相状態
	 
	 // 位相調整用パラメータ
    // 24'h800000 = 180度, 24'h400000 = 90度, 24'h200000 = 45度 相当
    // 必要に応じてこの値を書き換えて位相を微調整してください
    localparam [23:0]        PHASE_ADJUST    = 24'h000000; //24'h006883

    // ゲイン係数
    localparam	GAIN_SHFT = 10;     // 係数スケーリング用シフト幅
	 
	 // 異常検知関連
    localparam [23:0] UNLOCK_LIMIT = 24'd10_000_000; // 0.2秒 (clk_pll 50MHz時)

    //============================================================
    // 内部レジスタ・信号
    //============================================================
    wire                               clk_pll;
    wire                               pll_locked;
    
    // NCO (Numerical Controlled Oscillator) 関連
    reg  [PHASE_WIDTH+FRAC_WIDTH-1:0]	phase = 0;
	 // freq_incr: [24:8]が整数部, [7:0]が小数部
    reg  signed [16+FRAC_WIDTH:0]		freq_incr = INITIAL_INCR;
    wire [PHASE_WIDTH-1:0]            	phase_int = phase[PHASE_WIDTH+FRAC_WIDTH-1:FRAC_WIDTH];

    // 入力同期・エッジ検出
    reg                                in_d1, in_d2;
    wire                               in_edge = (in_d2 && !in_d1);
	 
    // 制御ループ (PD制御)
    wire signed [15:0]                 phase_error;
    reg  signed [15:0]                 phase_error_prev;
    reg  signed [15:0]                 phase_error_delta;
    reg  signed [15:0]                 locked_error_reg; // 誤差を保持

    // ゲイン選択・演算用
	// KP_FAST: 64 (2^6) 引き込み時 比例ゲイン
    // KD_FAST: 32 (2^5) 引き込み時 微分ゲイン
    // KP_SLOW: 16 (2^4) ロック後 比例ゲイン
	// KD_SLOW: 4  (2^2) ロック後 微分ゲイン
    wire signed [24:0]                 p_prod = (locked) ? (phase_error_prev << 4)  : (phase_error_prev << 6);
    wire signed [24:0]                 d_prod = (locked) ? (phase_error_delta << 2) : (phase_error_delta << 5);
    wire signed [24:0]                 p_term = p_prod >>> GAIN_SHFT;
    wire signed [24:0]                 d_term = d_prod >>> GAIN_SHFT;
    
    reg  signed [24:0]                 next_val; // 飽和処理用の中間変数
	 
	// 異常検知・出力保護用
    reg  [23:0]                        unlock_counter; // アンロック継続カウンタ
    reg                                force_low;      // 強制Loフラグ

    //============================================================
    // コンポーネント・演算ロジック
    //============================================================
    
    // PLLインスタンス
	 // clk:12MHz
	 // clk_pll:50MHz
    pll pll1 (
        .areset (1'b0),
        .inclk0 (clk),
        .c0     (clk_pll),
        .locked (pll_locked)
    );

    // 位相誤差計算
    assign phase_error = (|phase_int[31:25]) ? 16'h7FFF : (phase_int[24:9] - PHASE_OFFSET);

    //============================================================
    // メイン同期プロセス
    //============================================================
    always @(posedge clk_pll) begin
	    // 外部リセット(rst) または PLLが不安定な場合に同期リセット
        if (!rst || !pll_locked) begin
            // --- 全レジスタの初期化 ---
            phase             <= 0;
            freq_incr         <= INITIAL_INCR;
            in_d1             <= 0;
            in_d2             <= 0;
            
            // 制御ループ関連のリセット
            phase_error_prev  <= 0;
            phase_error_delta <= 0;
            locked_error_reg  <= 0;
            
            // 異常検知・出力保護関連のリセット
            unlock_counter    <= 0;
            force_low         <= 0;
            
            // 出力信号のリセット
				locked				<= 0;
            sq_out_0          <= 0;
            sq_out_90         <= 0;
        end 
        else begin
            // --- 1. 入力信号の同期処理 ---
            in_d1 <= in;
            in_d2 <= in_d1;

            // --- 2. 位相蓄積 (NCO) ---
            phase <= phase + freq_incr;

            // --- 3. 位相更新・ループ制御 (立ち下がりエッジ検出時) ---
            if (in_edge) begin
                // 誤差情報の更新
                phase_error_prev  <= phase_error;
                phase_error_delta <= phase_error - phase_error_prev;
					 
                // 誤差を記録
                locked_error_reg <= phase_error;
                
                // 周波数増分(freq_incr)の更新と飽和処理
					 // 元の制御値を「小数なし」として扱い、freq_incrにそのまま加算
					 // これにより、最下位bitが1/256 の重みとして作用する
                next_val = freq_incr - p_term + d_term;
                
                if (next_val < FREQ_INCR_MIN)
                    freq_incr <= FREQ_INCR_MIN;
                else if (next_val > FREQ_INCR_MAX)
                    freq_incr <= FREQ_INCR_MAX;
                else
                    freq_incr <= next_val;

                // 位相リセット（強制同期）
                phase <= 0;
            end

            // --- 4. ロック状態判定 ---
            // 誤差が一定範囲内 (-256 ~ +255) かどうか
            locked <= (locked_error_reg[15:8] == 8'b11111111 || locked_error_reg[15:8] == 8'b00000000);
				
				// アンロック継続時間の監視
            if (!locked) begin
                if (unlock_counter < UNLOCK_LIMIT) begin
                    unlock_counter <= unlock_counter + 1'b1;
                end else begin
                    force_low <= 1'b1; // 一定期間アンロックで出力を遮断
						  
						  // アンロック確定時に周波数をリセット
                    if (!force_low) begin
                        freq_incr <= INITIAL_INCR;
                    end
                end
            end else begin
                unlock_counter <= 0;
                force_low      <= 1'b0; // ロックすれば解除
            end

            // --- 5. 出力波形生成 ---
            if (force_low) begin
                sq_out_0  <= 1'b0;
                sq_out_90 <= 1'b0;
            end else begin
					 sq_out_0  <= ~((phase_int[23:0] + PHASE_ADJUST) >> 23);
                sq_out_90 <= ~((phase_int[23:0] + PHASE_90_OFFSET + PHASE_ADJUST) >> 23);
            end
        end
    end
endmodule