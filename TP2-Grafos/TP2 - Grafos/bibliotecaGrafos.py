def ordem(grafo):
  return len(grafo) - 1 #Retorna o tamanho da matriz - 1, pois a matriz é indexada de 1 a n

def tamanho(grafo):
  tamanho = 0
  for linha in grafo:
    for i in range(1, ordem(grafo)+1):
      if linha[i] != 0: #Verifica se existe uma aresta entre os vértices
        tamanho += 1
  return tamanho // 2 # A divisão por 2 é necessária para evitar a contagem de arestas duplicadas em grafos não direcionados.

def densidade(grafo):
  return tamanho(grafo)/ordem(grafo)

def vizinhos(grafo, vertice):
  vizinhos = []
  for i in range(1, ordem(grafo)+1):
    if grafo[vertice][i] != 0: #Verifica se existe uma aresta entre os vértices, verificando se o peso é diferente de 0
      vizinhos.append(i) #Adiciona o vértice à lista de vizinhos
  return vizinhos

def grauVertice(grafo, vertice):
  return len(vizinhos(grafo, vertice)) #Retorna o tamanho da lista de vizinhos, logo o grau do vértice

# A função usa busca em profundidade para explorar o grafo e identifica se a remoção do vértice desconecta componentes
def verificaArticulacao(grafo, vertice):
  def dfs(grafo, vertice, visitados): #Busca em profundidade
    visitados[vertice] = True
    for vizinho in vizinhos(grafo, vertice):
      if not visitados[vizinho]:
        dfs(grafo, vizinho, visitados)

  buscaOriginal =  [False] * (ordem(grafo) + 1)
  dfs(grafo, 1, buscaOriginal)

 # Marca o vértice de articulação como visitado
  visitados = [False] * (ordem(grafo) + 1)
  visitados[vertice] = True  

  # Realiza a busca em profundidade após a remoção do vértice
  if vertice == 1:
    dfs(grafo, 2, visitados)
  else:
    dfs(grafo, 1, visitados)

  # Verifica se a remoção do vértice desconecta o grafo
  for i in range(1, ordem(grafo) + 1):
    if visitados[i] != buscaOriginal[i] and i != vertice:
      return True  # O vértice é um ponto de articulação
  return False

def bfs(grafo, vertice, operacao):  # Busca em largura considerando toda a floresta
 
  visitados = [False] * (ordem(grafo) + 1)
  
  florestasOrdemVisita = []
  arestasRetorno = []
  arestasNormal = []

  # Função interna para realizar a busca em largura a partir de um vértice específico
  def bfs_componente(vertice):
    fila = [vertice]  
    visitados[vertice] = True  
    ordemVisita = [vertice] 

    while fila: 
      atual = fila.pop(0) 
      for vizinho in vizinhos(grafo, atual):  
        if not visitados[vizinho]: 
          visitados[vizinho] = True  
          fila.append(vizinho) 
          ordemVisita.append(vizinho) 
          arestasNormal.append((atual, vizinho))  # Adiciona a aresta normal
        else:
          # Se o vizinho já foi visitado e não é o vértice inicial, adiciona como aresta de retorno
          if (atual != vertice and vizinho != vertice and 
              (vizinho, atual) not in arestasRetorno and 
              (vizinho, atual) not in arestasNormal):
            arestasRetorno.append((atual, vizinho))

    florestasOrdemVisita.append(ordemVisita)  # Adiciona a ordem de visita da componente à lista de florestas

  # Inicia a busca a partir do vértice escolhido
  bfs_componente(vertice)

  # Continua a busca para os vértices não visitados
  for vertice in range(1, ordem(grafo) + 1):
    if not visitados[vertice]: 
      bfs_componente(vertice)  # Realiza a busca em largura a partir desse vértice

  if operacao == 1:
    return florestasOrdemVisita  
  elif operacao == 2:
    return arestasRetorno 
  else:
    print("Ordem de visita por componente:", florestasOrdemVisita)
    print("Arestas de retorno:", arestasRetorno)
    

def componentesConexas(grafo):
  visitados = [False] * (ordem(grafo) + 1)
  componentes = [] # Lista para armazenar as componentes

  def dfs(grafo, vertice, componente):
    visitados[vertice] = True
    componente.append(vertice)
    for vizinho in vizinhos(grafo, vertice):
      if not visitados[vizinho]:
        dfs(grafo, vizinho, componente)

  for vertice in range(1, ordem(grafo) + 1):
    if not visitados[vertice]:
      componente = []
      dfs(grafo, vertice, componente)
      componentes.append(componente) # Adiciona a componente encontrada.

  return componentes

# Retorna a quantidade de componentes conexas no grafo.
def qtdComponentesConexas(grafo):
  return len(componentesConexas(grafo)) #Retorna o tamanho da lista de componentes  


def possuiCiclo(grafo):
  def dfs(grafo, vertice, visitados, pai):  # Busca em profundidade
    visitados[vertice] = True
    for vizinho in vizinhos(grafo, vertice):
      if not visitados[vizinho]:  # Explora apenas vértices não visitados
        if dfs(grafo, vizinho, visitados, vertice):  # Passa o vértice atual como pai
          return True
      elif vizinho != pai:  # Vizinho já visitado que não é o pai indica ciclo
          return True
    return False

  visitados = [False] * (ordem(grafo) + 1)  # Assumindo vértices indexados de 1 a n

  for vertice in range(1, ordem(grafo) + 1):  # Lida com componentes desconexas
    if not visitados[vertice]:
      if dfs(grafo, vertice, visitados, -1):  # Usa -1 como pai inicial
        return True
  return False

def floyd_warshall(grafo):
    n = ordem(grafo)  # Número de vértices
    # Inicialização das matrizes L (menores distâncias) e R (reconstrução de caminhos)
    L = [[float('inf')] * n for _ in range(n)]
    R = [[None] * n for _ in range(n)]

    # Preenchendo L e R com os valores iniciais a partir do grafo
    for i in range(1, n + 1):
        for j in range(1, n + 1):
            if i == j:
                L[i - 1][j - 1] = 0  # Distância para si mesmo é 0
            elif grafo[i][j] != 0:
                L[i - 1][j - 1] = grafo[i][j] # Distância de arestas existentes

    # Regra de inicialização da matriz R
    for i in range(n):
        for j in range(n):
            if L[i][j] == float('inf'):  # Não há aresta entre i e j
                R[i][j] = None
            else:  # Inicialmente, o caminho mais curto é direto de i para j
                R[i][j] = i
    
    # Algoritmo de Floyd-Warshall
    for k in range(n):  # Índices intermediários (de 0 a n-1)
        for i in range(n):  # Vértices de origem (de 0 a n-1)
            for j in range(n):  # Vértices de destino (de 0 a n-1)
                if L[i][j] > L[i][k] + L[k][j] and R[i][k] != j and R[k][j] != i:
                    L[i][j] = L[i][k] + L[k][j]
                    R[i][j] = R[k][j]  # Atualiza o caminho para passar por k

    # Verificação de ciclos negativos
    for i in range(n):
        if L[i][i] < 0:
            print(f"Ciclo negativo detectado no vértice {i}")
            return None, None

    return L, R

def reconstruir_caminho(R, start, end):
    """Reconstrói o caminho mínimo de start a end usando a matriz R"""
    if R[start][end] is None:
        return None  # Não há caminho entre os vértices
    
    caminho = []
    atual = end
    while atual != start:
        caminho.append(atual + 1)  # Adiciona o vértice atual ao caminho (ajusta para a indexação 1)
        atual = R[start][atual]  # Move para o predecessor do vértice atual
    caminho.append(start + 1)  # Adiciona o vértice de origem ao caminho
    caminho.reverse()  # Reverte a lista para ter o caminho na ordem correta (do início ao fim)
    return caminho  # Retorna o caminho reconstruído

def obter_caminhos_e_distancias(grafo, vertice):
    
    L, R = floyd_warshall(grafo)
    if L is None or R is None:
        return None
    vertice -= 1  # Ajusta o índice do vértice para a matriz L
    """Retorna as distâncias e os caminhos mínimos do vértice dado para todos os outros"""
    caminhos = {}
    n = ordem(grafo)  
    for destino in range(n):
        
        caminho = reconstruir_caminho(R, vertice, destino) # Reconstrói o caminho do vértice de origem ao de destino
        # Foi utilizado um dicionario devido a sua facilidade para diferenciar as duas informações retornadas
        caminhos[destino] = {
            "distancia": L[vertice][destino],
            "caminho": caminho
        }
    return caminhos # Retorna o dicionário com as distâncias e caminhos mínimos para todos os vértices


    
  
def arvore_geradora_minima(grafo):
    n = ordem(grafo)  # Número de vértices
    verticesArvore = set()  # Começa vazio
    foraArvore = set(range(1, n+1)) # Inicialmente, todos os vértices estão fora da árvore
    arestasArvore = []  # Arestas da árvore geradora mínima
   
    def prim_componente(vertice_inicial):
        verticesArvore.add(vertice_inicial)

        while len(verticesArvore) < n and foraArvore:
            min_peso = float('inf')
            aresta = None

            for i in verticesArvore:
                for j in foraArvore:
                    if grafo[i][j] != 0 and grafo[i][j] < min_peso:  # Ajuste na indexação
                        min_peso = grafo[i][j]
                        aresta = (i, j)
            
            if not aresta:
                break  # Se não houver mais arestas, o grafo é desconexo

            i, j = aresta
            verticesArvore.add(j)  # Adiciona vértice à árvore
            foraArvore.remove(j)  # Remove vértice do conjunto fora da árvore
            arestasArvore.append(aresta)  # Adiciona aresta à solução

    while len(foraArvore) > 0:
        # Pega um vértice do conjunto foraArvore
        vertice_inicial = foraArvore.pop()
        

        # Executa o algoritmo de Prim a partir desse vértice
        prim_componente(vertice_inicial)

    pesototal = 0
    with open('/home/leticia/Documentos/Grafos/TP2-Grafos/TP2 - Grafos/arvoreGeradoraMinima.txt', 'w') as f:
      f.write(f"{len(arestasArvore)}\n") # Escreve a quantidade de arestas
      for aresta in arestasArvore:
        pesototal += grafo[aresta[0]][aresta[1]] # Soma os pesos das arestas
        f.write(f"{aresta[0]} {aresta[1]} {grafo[aresta[0]][aresta[1]]}\n")  # Escreve a aresta e seu peso
      f.write(f"{pesototal}") 
    return arestasArvore


def determina_centralidade_grafo(grafo, vertice):
  resultados = obter_caminhos_e_distancias(grafo, vertice) #Obtém os caminhos e distâncias
  somador = 0
  contador_desconexos = 0
  if resultados is None: # Verifica se há ciclos negativos
    return None
  else:
    for destino, info in resultados.items():#Percorre o dicionário de resultados
      if (info['distancia'])==float('inf'):   #Verifica se a distância é infinita 
        contador_desconexos += 1 #Incrementa o contador de desconexos do vertice
        continue
      somador += (info['distancia'])# Soma as distâncias dos caminhos mínimos

    ordemGrafo = ordem(grafo) - 1 - contador_desconexos #Obtém a quantidade de vertices do grafo menos 1 e menos a quantidade de vertices que estao desconectados
    centralidade = ordemGrafo/somador 
    return centralidade


def cobertura_minima(grafo): # Heurística de Greedy
    n = len(grafo)  # Número de vértices no grafo
    cobertura = set()  # Conjunto de vértices da cobertura mínima
    arestas_nao_cobertas = set()  # Conjunto de arestas não cobertas

    # Adicionar todas as arestas à lista de arestas não cobertas
    for i in range(1, n):  # Começa do vértice 1 até n
        for j in range(i, n):  # Considera a metade superior da matriz
            if grafo[i][j] != 0:  # Se há uma aresta entre i e j
                arestas_nao_cobertas.add((i, j))
    
    while arestas_nao_cobertas:
        # Calcular o grau de cada vértice
        graus = []
        for i in range(1, n):
            graus.append((i, grauVertice(grafo, i)))  # Armazena o vértice e seu grau
            
        # Encontrar o vértice com o maior grau
        vertice = max(graus, key=lambda x: x[1])[0]  # Pegamos o vértice com o maior grau

        # Adicionar o vértice à cobertura
        cobertura.add(vertice)

        # Remover as arestas incidentes ao vértice da lista de arestas não cobertas
        for i in range(1, n):
            for j in range(1, n):
                if grafo[i][j] != 0 and (i == vertice or j == vertice):
                    arestas_nao_cobertas.discard((i, j))  # Remove a aresta da lista
                    grafo[i][j] = 0  # Remove a ligação da matriz de adjacência
                    grafo[j][i] = 0  # Como a matriz é simétrica, também removemos a ligação reversa

    
    return len(cobertura), cobertura

