tree = [
 [3, 12, 8], # A (MIN node)
 [8, 4, 6], # B (MIN node)
 [14, 5, 2] # C (MIN node)
]

def minimax(node, is_max, alpha, beta):

   # If node is a leaf (number), return its value
   if isinstance(node, int):
      return node

   # MAX player
   if is_max:
       best = float('-inf')
       for child in node:
           value = minimax(child, False, alpha, beta)
           best = max(best, value)
           alpha = max(alpha, best)
           if best >= beta:
            break
       return best

   # MIN player
   else:
       best = float('inf')
       for child in node:
            value = minimax(child, True, alpha, beta)
            best = min(best, value)
            beta = min(beta, best)  
            if best <= alpha:
                break        
       return best

# Run minimax starting from root (MAX)
result = minimax(tree, True, float('-inf'), float('inf'))
print("Best value for MAX:", result)