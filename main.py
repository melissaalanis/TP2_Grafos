import os

from bibliotecaGrafos import *

def leituraArquivo(nome):
  grafo = []
  try:
    with open(nome, 'r') as arquivo:
      tamanho = int(arquivo.readline().strip()) # Remove espaços em branco do início e do fim da string
      grafo = [[0 for _ in range(tamanho + 1)] for _ in range(tamanho + 1)]

      linhas =  arquivo.readlines() # Lê todas as linhas do arquivo
      for linha in linhas:
        inicio, fim, peso = map(str, linha.split()) # Separa os valores da linha
        inicio = int(inicio) 
        fim = int(fim) 
        peso = float(peso)
        grafo[inicio][fim] = peso # Adiciona o peso da aresta no grafo (duuas vezes, porque é um grafo não direcionado)
        grafo[fim][inicio] = peso

  except FileNotFoundError:
    print("Erro: O arquivo não foi encontrado.")
    return None 
  except Exception as nome:
    print(f"Erro ao ler o arquivo: {nome}")
    return  None

  return grafo

def verificaValorVertice(grafo, vertice):
  if(vertice > ordem(grafo) or vertice < 0):
    return 1 # indica que foi informado um vertice que esta fora do intervalo do grafo
  return 0

def main():

  print("Bem-vindo(a) à biblioteca de grafos não direcionados ponderados!\n")
  while True:
      nome_arquivo = input("Digite o caminho do arquivo: ").strip() # Remove espaços em branco do início e do fim da string
      grafo = leituraArquivo(nome_arquivo)

      if grafo is not None: # Se o grafo foi carregado com sucesso, interrompe o loop
          print("Grafo carregado com sucesso!")
          break
      else:
          print("Tente novamente!")

  opcao = int(1)
  while (opcao != 0):
      print("\nEscolha uma opção:")
      print("1 - Retornar a ordem do grafo")
      print("2 - Retornar o tamanho do grafo")
      print("3 - Retornar a densidade ε(G) do grafo")
      print("4 - Retornar os vizinhos de um vértice fornecido")
      print("5 - Retornar o grau de um vértice fornecido")
      print("6 - Verificar se um vértice é articulação")
      print("7 - Retornar a sequência de vértices visitados na busca em largura e informar a(s) aresta(s) que não faz(em) parte da árvore de busca em largura")
      print("8 - Retornar o número de componentes conexas do grafo e os vértices de cada componente")
      print("9 - Verificar se um grafo possui ciclo")
      print("10 - Retornar a distância e o caminho mínimo")
      print("11 - Retornar a árvore geradora mínima")
      print("12 - Retornar a cobertura mínima do grafo")
      print("13 - Retornar a centralidade de proximidade de um vértice")
      print("14 - Retornar emparelhamento máximo do grafo")
      print("0 - Sair\n")

      opcao = int(input("Digite a opção desejada: "))

      match opcao:
            case 0:
                break
            case 1:
                print("Ordem do grafo:", ordem(grafo))
            case 2:
                print("Tamanho do grafo:", tamanho(grafo))
            case 3:
                print(f"Densidade do grafo: {densidade(grafo):.2f}")

            case 4:
                vertice = int(input("Digite o vértice: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice: "))
                print(f"Vizinhos do vértice {vertice}: {vizinhos(grafo, vertice)}")

            case 5:
                vertice = int(input("Digite o vértice: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice: "))
                print(f"Grau do vértice {vertice}: {grauVertice(grafo, vertice)}")

            case 6:
                vertice = int(input("Digite o vértice: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice: "))
                if(verificaArticulacao(grafo, vertice)):
                    print(f"O vértice {vertice} é uma articulação")
                else:
                    print(f"O vértice {vertice} não é uma articulação")

            case 7:
                vertice = int(input("Digite o vértice pelo qual deseja iniciar a busca: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice pelo qual deseja iniciar a busca: "))
                print(f"Busca em largura a partir do vértice {vertice}: ")
                bfs(grafo, vertice, 3)

            case 8:
                componentes = componentesConexas(grafo)
                print("Número de componentes conexas:", len(componentes))
                for i, componente in enumerate(componentes):
                    print(f"Componente {i+1}: {componente}")

            case 9:
                if(possuiCiclo(grafo)):
                    print("O grafo possui ciclo")
                else:
                    print("O grafo não possui ciclo")

            case 10:
                vertice = int(input("Digite o vértice: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice: "))
                resultados = obter_caminhos_e_distancias(grafo, vertice)
                if resultados is None:
                    print("O grafo possui ciclo negativo.")
                else:
                    print(f"\nDistâncias e caminhos a partir do vértice {vertice}:")
                    for destino, info in resultados.items():
                        print(f"Para o vértice {destino + 1}:")
                        print(f"  Distância: {info['distancia']:.2f}")
                        print(f"  Caminho: {info['caminho']}")
            case 11:
                print("Árvore geradora mínima: ")
                arvore = arvore_geradora_minima(grafo)
                for aresta in arvore:
                    print(aresta)
                
            case 12:
                tamanho_cobertura, cobertura = cobertura_minima(grafo)
                print(f"Tamanho da cobertura mínima: {tamanho_cobertura}")
                print(f"Cobertura mínima: {cobertura}")
            case 13:
                vertice = int(input("Digite o vértice: "))
                while(verificaValorVertice(grafo, vertice)):
                    print("Valor Inválido! Por favor digite um número que esteja no intervalo do grafo")
                    vertice = int(input("Digite o vértice: "))
                if qtdComponentesConexas(grafo) > 1:
                    print("O grafo não é conexo, calculando centralidade da componete conexa...")

                resultados = determina_centralidade_grafo(grafo, vertice)
                
                if resultados is None:
                    print("O grafo possui ciclo negativo. Não é possível calcular a centralidade de proximidade.")
                else:
                    print(f"Centralidade de proximidade do vertice {vertice} é: {resultados}")

            case 14:
                pares, peso = encontra_emparelhamento_maximo(grafo)
                if pares is not None:  # Se encontrou um emparelhamento
                    print("\nEmparelhamento Máximo encontrado:")
                    print(pares)
                    print(f"Tamanho do emparelhamento: {len(pares)}")
                    print(f"Peso total do emparelhamento: {peso}\n") 
            case _:
                print("Opção inválida. Tente novamente.")
              
      input("Pressione Enter para continuar...")
      os.system('cls' if os.name == 'nt' else 'clear')

if __name__ == "__main__":
  main()