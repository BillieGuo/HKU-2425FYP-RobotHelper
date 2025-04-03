from geometry_msgs.msg import Point

class QueryPointProcessor:
    def __init__(self):
        self.query_points = []
        self.feat_similarities = []
        self.service_names = []
        self.labels = []
        self.default_strategy = self.max_similarity
    
    def update_values(self, query_points, feat_similarities, service_names, labels):
        self.query_points = query_points
        self.feat_similarities = feat_similarities
        self.service_names = service_names
        self.labels = labels
    
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
        print("Sorted by similarity")
        for i in range(len(self.feat_similarities)):
            print(f"Point: {self.query_points[i]}, Feature Similarity: {self.feat_similarities[i]}, Service Name: {self.service_names[i]}, Label: {self.labels[i]}")
        return self.query_points, self.feat_similarities, self.service_names, self.labels

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

        
        