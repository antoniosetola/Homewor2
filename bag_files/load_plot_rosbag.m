% Load the bag file
bag = rosbag('first_circ_pol.bag');

topics = { '/iiwa/iiwa_joint_1_effort_controller/command',...
    '/iiwa/iiwa_joint_2_effort_controller/command',...
    '/iiwa/iiwa_joint_3_effort_controller/command',...
    '/iiwa/iiwa_joint_4_effort_controller/command',...
    '/iiwa/iiwa_joint_5_effort_controller/command',...
    '/iiwa/iiwa_joint_6_effort_controller/command',...
    '/iiwa/iiwa_joint_7_effort_controller/command'};

% Prealloca le celle per i dati
dataValues = cell(1, numel(topics));

for i = 1:numel(topics)
    data = readMessages(select(bag, 'Topic', topics{i}));
    dataValues{i} = cellfun(@(msg) msg.Data, data);
end

% Plot data for each topic
figure;

for i = 1:numel(topics)
    plot(dataValues{i}, 'DisplayName', topics{i});
    hold on;
end

title('Joint Torque Commands');
xlabel('Time');
ylabel('Joint Torque');
grid on;
legend('show');
