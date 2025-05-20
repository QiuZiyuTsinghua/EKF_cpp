% 直接从源代码编译EKF S-Function
fprintf('直接从源代码编译EKF S-Function...\n');

% 配置路径
EKF_DIR = 'C:\Users\ziyu.qiu\Documents\code\cpp\EKF_cpp';  % EKF源代码目录
EIGEN_DIR = 'C:\Users\ziyu.qiu\Documents\lib\eigen-3.4.0';             % Eigen库目录

% 检查源文件是否存在
ekf_h_file = fullfile(EKF_DIR, 'ekf.h');
ekf_cpp_file = fullfile(EKF_DIR, 'ekf.cpp');

if ~exist(ekf_h_file, 'file')
    error('找不到ekf.h文件，请检查路径: %s', ekf_h_file);
end

if ~exist(ekf_cpp_file, 'file')
    error('找不到ekf.cpp文件，请检查路径: %s', ekf_cpp_file);
end

fprintf('使用以下配置:\n');
fprintf('  - EKF源代码路径: %s\n', EKF_DIR);
fprintf('  - Eigen库路径: %s\n', EIGEN_DIR);
fprintf('  - ekf.h位置: %s\n', ekf_h_file);
fprintf('  - ekf.cpp位置: %s\n', ekf_cpp_file);

% 直接编译S-Function和EKF源代码 - 不使用预编译库
try
    cmd = ['mex -v ', ...
           '-I"', EKF_DIR, '" ', ...
           '-I"', EIGEN_DIR, '" ', ...
           '"', fullfile(pwd, 'ekf_sfun.cpp'), '" ', ...
           '"', ekf_cpp_file, '"'];
    
    fprintf('执行命令: %s\n', cmd);
    eval(cmd);
    
    fprintf('\n编译成功! 已生成ekf_sfun.mexw64文件\n');
catch ME
    fprintf('\n编译失败: %s\n', ME.message);
    
    % 提供更详细的错误信息和建议
    if contains(lower(ME.message), 'eigen')
        fprintf('\n可能是Eigen库路径问题。请确认路径下有Eigen目录，包含Dense等头文件。\n');
        fprintf('Eigen目录应该是: %s\n', fullfile(EIGEN_DIR, 'Eigen'));
    elseif contains(lower(ME.message), 'ekf.h') || contains(lower(ME.message), 'ekf.cpp')
        fprintf('\n找不到EKF源文件。请确认ekf.h和ekf.cpp在指定路径中。\n');
    else
        fprintf('\n可能是编译器问题。请运行"mex -setup C++"确认已安装C++编译器。\n');
    end
end

% 使用说明
fprintf('\nSimulink中使用方法:\n');
fprintf('1. 添加S-Function模块到模型\n');
fprintf('2. 设置S-Function名称为"ekf_sfun"\n');
fprintf('3. 配置以下参数:\n');
fprintf('   - 参数1: STATE_DIM (状态向量维度)\n');
fprintf('   - 参数2: MEAS_DIM (测量向量维度)\n');
fprintf('   - 参数3: CTRL_DIM (控制输入维度，无则为0)\n');
fprintf('   - 参数4: DT (时间步长，秒)\n');
fprintf('   - 参数5: INITIAL_STATE (初始状态向量)\n');
fprintf('   - 参数6: INITIAL_COV (初始协方差矩阵)\n');
fprintf('   - 参数7: PROCESS_NOISE_COV (过程噪声协方差矩阵)\n');
fprintf('   - 参数8: MEAS_NOISE_COV (测量噪声协方差矩阵)\n');