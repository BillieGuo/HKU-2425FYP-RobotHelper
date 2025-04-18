from geometry_msgs.msg import Point

class QueryPointProcessor:
    def __init__(self):
        self.query_points = []
        self.feat_similarities = []
        self.service_names = []
        self.labels = []
        self.confs = []
        self.default_strategy = self.max_similarity
        self.sim_1_threshold = 0.95
    
    def update_values(self, query_points, feat_similarities, service_names, labels, confs):
        self.query_points = query_points
        self.feat_similarities = feat_similarities
        self.service_names = service_names
        self.labels = labels
        self.confs = confs
    
    def filter_confidence(self, confidence_threshold=0.5):
        '''
        Filter the query_points, feat_similarities, service_names, labels by confidence_threshold
        Usage:
        query_point_processor.filter_confidence(confidence_threshold=0.5)
        '''
        query_points = []
        feat_similarities = []
        service_names = []
        labels = []
        confs = []
        for i in range(len(self.confs)):
            if self.confs[i] > confidence_threshold:
                query_points.append(self.query_points[i])
                feat_similarities.append(self.feat_similarities[i])
                service_names.append(self.service_names[i])
                labels.append(self.labels[i])
                confs.append(self.confs[i])
        self.query_points = query_points
        self.feat_similarities = feat_similarities
        self.service_names = service_names
        self.labels = labels
        self.confs = confs
        print("Filtered by confidence")

    def sort_similarity(self):
        '''
        Sort the query_points, feat_similarities, service_names, labels by feat_similarities
        Usage:
        query_point_processor.sort_similarity()
        '''
        sorted_indices = sorted(range(len(self.feat_similarities)), key=lambda k: self.feat_similarities[k], reverse=True)
        self.query_points = [self.query_points[i] for i in sorted_indices]
        self.feat_similarities = [self.feat_similarities[i] for i in sorted_indices]
        self.service_names = [self.service_names[i] for i in sorted_indices]
        self.labels = [self.labels[i] for i in sorted_indices]
        self.confs = [self.confs[i] for i in sorted_indices]
        print("Sorted by similarity")
        for i in range(len(self.feat_similarities)):
            print(f"Point: {self.query_points[i]}, Feature Similarity: {self.feat_similarities[i]}, Service Name: {self.service_names[i]}, Label: {self.labels[i]}")
        return self.query_points, self.feat_similarities, self.service_names, self.labels, self.confs
    
    def sim_1_sort_confs(self):
        '''
        for elements with similarity 1 sort the confidences in descending order.
        Usage:
        query_point_processor.sim_1_sort_confs()
        '''
        query_points = []
        feat_similarities = []
        service_names = []
        labels = []
        confs = []
        for i in range(len(self.feat_similarities)):
            if self.feat_similarities[i] >= self.sim_1_threshold:
                query_points.append(self.query_points[i])
                feat_similarities.append(self.feat_similarities[i])
                service_names.append(self.service_names[i])
                labels.append(self.labels[i])
                confs.append(self.confs[i])
        sorted_indices = sorted(range(len(confs)), key=lambda k: confs[k], reverse=True)
        query_points = [query_points[i] for i in sorted_indices]
        feat_similarities = [feat_similarities[i] for i in sorted_indices]
        service_names = [service_names[i] for i in sorted_indices]
        labels = [labels[i] for i in sorted_indices]
        confs = [confs[i] for i in sorted_indices]
        return query_points, feat_similarities, service_names, labels, confs
    
    def sort_conf_plus_original(self):
        '''
        Sort the query_points, feat_similarities, service_names, labels by confidences in descending order, then sort the original query_points, feat_similarities, service_names, labels by feat_similarities in descending order.
        Usage:
        query_point_processor.sort_conf_plus_original()
        '''
        self.sort_similarity()
        query_points, feat_similarities, service_names, labels, confs = self.sim_1_sort_confs()
        self.query_points = query_points + self.query_points
        self.feat_similarities = feat_similarities + self.feat_similarities
        self.service_names = service_names + self.service_names
        self.labels = labels + self.labels
        self.confs = confs + self.confs
        # print("Sorted by confidences in descending order, then sort the original query_points, feat_similarities, service_names, labels by feat_similarities in descending order.")
        # for i in range(len(self.feat_similarities)):
        #     print(f"Point: {self.query_points[i]}, Feature Similarity: {self.feat_similarities[i]}, Service Name: {self.service_names[i]}, Label: {self.labels[i]}")
        return self.query_points, self.feat_similarities, self.service_names, self.labels, self.confs

    def max_of_sim_1_sort_confs(self):
        '''
        Return the max similarity point and the corresponding feature_similarity, service_name, label of the elements with similarity 1.
        Usage:
        query_point_processor.max_of_sim_1_sort_confs()
        '''
        query_points, feat_similarities, service_names, labels, confs = self.sim_1_sort_confs()
        print(query_points, feat_similarities, service_names, labels, confs)
        if query_points == []:
            return None, None, None, None, None
        else:
            return query_points[0], feat_similarities[0], service_names[0], labels[0], confs[0]

    def max_similarity(self):
        '''
        Return the max similarity point and the corresponding feature_similarity, service_name, label

        Usage: 
        query_point, feat_similarity, service_name, label= query_point_processor.max_similarity()
        '''
        print("Max similarity: ", max(self.feat_similarities))
        for i in range(len(self.feat_similarities)):
            if self.feat_similarities[i] == max(self.feat_similarities):
                return self.query_points[i], self.feat_similarities[i], self.service_names[i], self.labels[i]
        return None, None, None, None

    def max_similarities(self):
        '''
        Return the max similarity points and the corresponding feature_similarities, service_names, labels

        Usage:
        max_set_of_points = query_point_processor.max_similarities()
        for point, feat_similarity, service_name, label in max_set_of_points:
            print(f"Point: {point}, Feature Similarity: {feat_similarity}, Service Name: {service_name}, Label: {label}")
        '''
        max_set_of_points = []
        for i in range(len(self.feat_similarities)):
            if self.feat_similarities[i] == max(self.feat_similarities):
                max_set_of_points.append((self.query_points[i], self.feat_similarities[i], self.service_names[i], self.labels[i]))
        return max_set_of_points

        
        