from tf2_ros import Buffer


class BagBuffer(Buffer):
    def __init__(self, bag):
        msgs = [msg for _, msg, _ in bag.read_messages(topics=['/tf'])]
        tfs = [tf for msg in msgs for tf in msg.transforms]

        ts = [tf.header.stamp for tf in tfs]

        super(BagBuffer, self).__init__(cache_time=max(ts) - min(ts), debug=False)

        # fill the buffer with all tf messages from the bagfile
        for tf in tfs:
            self.set_transform(tf, 'rosbag')
